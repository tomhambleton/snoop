/* 
 * File:   snoopmon.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include <opencv2/core/core_c.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCamControl.h"

#include "vgfont.h"

#define GX 0 
#define PREVIEW 0 


#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 720
#define VIDEO_FPS 30 

#define BITRATE 1500000
#define CAPTURE_LENGTH 8  // Seconds
#define SUSPEND_LENGTH 25  // Seconds
#define MOTION_PERIOD (VIDEO_FPS/3)  // Times per second

#define CAPTURE_FRAME_COUNT (VIDEO_FPS*CAPTURE_LENGTH) 
#define SUSPEND_FRAME_COUNT ((VIDEO_FPS*SUSPEND_LENGTH)-MOTION_PERIOD)

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

#define ACTION_NULL 0
#define ACTION_CHECK_MOTION 1
#define ACTION_STOP_CAPTURE 2

#define STATE_NORMAL 0
#define STATE_CAPTURE 1
#define STATE_SUSPEND 2

typedef struct {
    int video_width;
    int video_height;
    int preview_width;
    int preview_height;
    int opencv_width;
    int opencv_height;
    int quality;
    MMAL_FOURCC_T encoding;
    float video_fps;
    MMAL_COMPONENT_T *camera;
    MMAL_COMPONENT_T *encoder;
    MMAL_COMPONENT_T *preview;
    MMAL_PORT_T *camera_preview_port;
    MMAL_PORT_T *camera_video_port;
    MMAL_PORT_T *camera_still_port;
    MMAL_POOL_T *camera_video_port_pool;
    MMAL_PORT_T *encoder_input_port;
    MMAL_POOL_T *encoder_input_pool;
    MMAL_PORT_T *encoder_output_port;
    MMAL_POOL_T *encoder_output_pool;
    RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters
    unsigned char* videoBuffer;
    int            videoBufferLen;
    CvMemStorage* storage;
    IplImage* image;
    IplImage* image1;
    IplImage* image2;
    IplImage* prevImage;
    IplImage* py1;
    IplImage* py2;
    VCOS_SEMAPHORE_T complete_semaphore;
    VCOS_SEMAPHORE_T filewrite_semaphore;
    VCOS_MUTEX_T     filewrite_lock;
    FILE* fptr;
    int  bufferAction;
    int  state;
    int  pendingState;
} PORT_USERDATA;

struct my_msgbuf {
    long mtype;
    char mtext[80];
};


/**
 *  buffer header callback function for video
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
#define MOTION_CLEAR_SOAK 1

int g_DiffThreshold = 25;
int g_NoiseWindow = 3; // must be odd
int g_PixelThreshold = 5000; // Total number of pixels changed

static int compareImages(PORT_USERDATA* userdata)
{
    int w = userdata->opencv_width;
    int h = userdata->opencv_height;
    unsigned char* buf0 = userdata->prevImage->imageData;
    unsigned char* buf1 = userdata->image2->imageData;
    int dataSize = w*h;
    int i=0;
    int j=0;
    int x=0;
    int y=0;
    for(i = 0; i<dataSize; i++) {
        int diff = buf0[i]-buf1[i];        
        if (diff < 0) diff *= -1;
        userdata->py1->imageData[i] = (diff >= g_DiffThreshold) ? 255: 0;
    }
    memset(userdata->py2->imageData, 0, dataSize);
    int m = (g_NoiseWindow*g_NoiseWindow)/2;
    int n = (g_NoiseWindow-1)/2;
    int total = 0;
    int w1 = w-1;
    int h1 = h-1;
    for (x=1; x<w1; x++) {
        for (y=1; y<h1; y++) {
            int marked = 0; 
            int xstop = x+n;
            int ystop = y+n;
            for (i=x-n; i<xstop+n; i++) {
                for (j=y-n; j<ystop; j++) {
                    if (userdata->py1->imageData[j*w+i] == 255) 
                        marked++;
                    if (marked >=m) break;
                }
                if (marked >=m) break;
            }
            if (marked >= m) {
                for(i=x-n; i<xstop; i++) {
                    for(j=y-n; j<ystop; j++) {
                        userdata->py2->imageData[j*w+i] = 255;
                        total++;
                    }
                }
            }
        }
    }
    return total;
}

static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    static int frame_count = 0;
    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA * userdata = (PORT_USERDATA *) port->userdata;
    MMAL_POOL_T *pool = userdata->camera_video_port_pool;

    if (userdata->state != userdata->pendingState) {
        userdata->state = userdata->pendingState;
        frame_count = 1;
    } else {
        frame_count++;
    }
    switch(userdata->state) {
        case STATE_NORMAL:
            if ((frame_count % MOTION_PERIOD) == 0) {
                userdata->bufferAction = ACTION_CHECK_MOTION;
                if (vcos_semaphore_trywait(&(userdata->complete_semaphore)) != VCOS_SUCCESS) {
                    mmal_buffer_header_mem_lock(buffer);
                    memcpy(userdata->videoBuffer, buffer->data, buffer->length);
                    userdata->videoBufferLen = buffer->length;
                    mmal_buffer_header_mem_unlock(buffer);
                    vcos_semaphore_post(&(userdata->complete_semaphore));  // Tell other thread to proc frame
                }
            }
            break;
        case STATE_CAPTURE:
            if (frame_count >= CAPTURE_FRAME_COUNT) {
                userdata->bufferAction = ACTION_STOP_CAPTURE;
                if (vcos_semaphore_trywait(&(userdata->complete_semaphore)) != VCOS_SUCCESS) {
                    mmal_buffer_header_mem_lock(buffer);
                    memcpy(userdata->videoBuffer, buffer->data, buffer->length);
                    userdata->videoBufferLen = buffer->length;
                    mmal_buffer_header_mem_unlock(buffer);
                    vcos_semaphore_post(&(userdata->complete_semaphore));  // Tell other thread to proc frame
                }
            } else {
                MMAL_BUFFER_HEADER_T *output_buffer = 0;
                output_buffer = mmal_queue_get(userdata->encoder_input_pool->queue);
                if (output_buffer) {
                    mmal_buffer_header_mem_lock(buffer);
                    memcpy(output_buffer->data, buffer->data, buffer->length);
                    output_buffer->length = buffer->length;
                    mmal_buffer_header_mem_unlock(buffer);
                    if (mmal_port_send_buffer(userdata->encoder_input_port, output_buffer) != MMAL_SUCCESS) {
                        fprintf(stderr, "ERROR: Unable to send buffer to encoder output\n");
                    }
                } else {
                    printf("Unable to get encoder input buffer!\n");
                }
            }
            break;
        case STATE_SUSPEND:
            if (frame_count >= SUSPEND_FRAME_COUNT) {
                userdata->pendingState = STATE_NORMAL;
                userdata->bufferAction = ACTION_NULL;
                if (vcos_semaphore_trywait(&(userdata->complete_semaphore)) != VCOS_SUCCESS) {
                    mmal_buffer_header_mem_lock(buffer);
                    memcpy(userdata->videoBuffer, buffer->data, buffer->length);
                    userdata->videoBufferLen = buffer->length;
                    mmal_buffer_header_mem_unlock(buffer);
                    vcos_semaphore_post(&(userdata->complete_semaphore));  // Tell other thread to proc frame
                }
            }
            break;
        default:
            printf("Unknown state: %d\n", userdata->state);
    }
    mmal_buffer_header_release(buffer);
    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            printf("Unable to return a buffer to the video port\n");
    }
}


static void encoder_input_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //fprintf(stderr, "INFO:%s\n", __func__);    
    //mmal_buffer_header_mem_lock(buffer);
    //printf("In encoder_input_buffer_callback, len = %d\n", buffer->length);
    //mmal_buffer_header_mem_unlock(buffer);
    mmal_buffer_header_release(buffer);
}

static void encoder_output_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA *userdata = (PORT_USERDATA *) port->userdata;
    MMAL_POOL_T *pool = userdata->encoder_output_pool;

    // printf("In encoder_output_buffer_callback, len = %d\n", buffer->length);
    if (userdata) {
        mmal_buffer_header_mem_lock(buffer);
        vcos_mutex_lock(&userdata->filewrite_lock);
        if (userdata->fptr) {
            fwrite(buffer->data, 1, buffer->length, userdata->fptr);
        }
        vcos_mutex_unlock(&userdata->filewrite_lock);
        mmal_buffer_header_mem_unlock(buffer);
    } 
    mmal_buffer_header_release(buffer);
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS) {
            fprintf(stderr, "Unable to return a buffer to the video port\n");
        }
    }
}

int setup_camera(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T * camera_preview_port;
    MMAL_PORT_T * camera_video_port;
    MMAL_PORT_T * camera_still_port;
    MMAL_POOL_T * camera_video_port_pool;
    // Set up the camera_parameters to default
    raspicamcontrol_set_defaults(&userdata->camera_parameters);

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: create camera %x\n", status);
        return -1;
    }
    userdata->camera = camera;
    userdata->camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    userdata->camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    userdata->camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];


    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
            .max_stills_w = VIDEO_WIDTH,
            .max_stills_h = VIDEO_HEIGHT,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = VIDEO_WIDTH,
            .max_preview_video_h = VIDEO_HEIGHT,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }

    // Setup camera preview port format 
    format = camera_preview_port->format;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = VIDEO_WIDTH;
    format->es->video.height = VIDEO_HEIGHT;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = VIDEO_WIDTH;
    format->es->video.crop.height = VIDEO_HEIGHT;
    format->es->video.frame_rate.num = VIDEO_FPS;
    format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(camera_preview_port);

    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: camera viewfinder format couldn't be set\n");
        return -1;
    }

    // Setup camera video port format
    mmal_format_copy(camera_video_port->format, camera_preview_port->format);

    format = camera_video_port->format;
    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = VIDEO_WIDTH;
    format->es->video.height = VIDEO_HEIGHT;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = VIDEO_WIDTH;
    format->es->video.crop.height = VIDEO_HEIGHT;
    format->es->video.frame_rate.num = VIDEO_FPS;
    format->es->video.frame_rate.den = 1;

    //camera_video_port->buffer_size = format->es->video.width * format->es->video.height * 12 / 8;
    camera_video_port->buffer_size = format->es->video.width * format->es->video.height * 2;
    camera_video_port->buffer_num = 2;  // or 2?
    userdata->videoBuffer = malloc(camera_video_port->buffer_size);
    userdata->videoBufferLen  = 0;

    fprintf(stderr, "INFO:camera video buffer_size = %d\n", camera_video_port->buffer_size);
    fprintf(stderr, "INFO:camera video buffer_num = %d\n", camera_video_port->buffer_num);

    status = mmal_port_format_commit(camera_video_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit camera video port format (%u)\n", status);
        return -1;
    }
    if (camera_video_port->buffer_num < 3)
        camera_video_port->buffer_num = 3;

    camera_video_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(camera_video_port, 
                                                                   camera_video_port->buffer_num, 
                                                                   camera_video_port->buffer_size);
    userdata->camera_video_port_pool = camera_video_port_pool;
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    status = mmal_port_enable(camera_video_port, video_buffer_callback);

    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable camera video port (%u)\n", status);
        return -1;
    }

    status = mmal_component_enable(camera);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable camera (%u)\n", status);
        return -1;
    }

    raspicamcontrol_set_all_parameters(camera, &userdata->camera_parameters);

    fprintf(stderr, "INFO: camera created\n");
    return 0;
}

static void fill_port_buffer(MMAL_PORT_T *port, MMAL_POOL_T *pool) {
    int q;
    int num = mmal_queue_length(pool->queue);

    for (q = 0; q < num; q++) {
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
        if (!buffer) {
            fprintf(stderr, "Unable to get a required buffer %d from pool queue\n", q);
        }

        if (mmal_port_send_buffer(port, buffer) != MMAL_SUCCESS) {
            fprintf(stderr, "Unable to send a buffer to port (%d)\n", q);
        }
    }
}


int setup_encoder(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *preview_input_port = NULL;

    MMAL_PORT_T *encoder_input_port = NULL, *encoder_output_port = NULL;
    MMAL_POOL_T *encoder_input_port_pool;
    MMAL_POOL_T *encoder_output_port_pool;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create preview (%u)\n", status);
        return -1;
    }
    userdata->encoder = encoder;
    encoder_input_port = encoder->input[0];
    encoder_output_port = encoder->output[0];
    userdata->encoder_input_port = encoder_input_port;
    userdata->encoder_output_port = encoder_input_port;

    mmal_format_copy(encoder_input_port->format, userdata->camera_video_port->format);
    encoder_input_port->buffer_size = encoder_input_port->buffer_size_recommended;
    /*
    if (encoder_input_port->buffer_size < encoder_input_port->buffer_size_min) {
        encoder_input_port->buffer_size = encoder_input_port->buffer_size_min;
    }
    */
    encoder_input_port->buffer_num = 3;
    /*
    if (encoder_input_port->buffer_num < encoder_input_port->buffer_num_min) {
        encoder_input_port->buffer_num = encoder_input_port->buffer_num_min;
    }
    */
    mmal_format_copy(encoder_output_port->format, encoder_input_port->format);
    // Commit the port changes to the input port 

    encoder_output_port->buffer_size = encoder_output_port->buffer_size_recommended;
    if (encoder_output_port->buffer_size < encoder_output_port->buffer_size_min) {
        encoder_output_port->buffer_size = encoder_output_port->buffer_size_min;
    }
    encoder_output_port->buffer_num = 2;
    if (encoder_output_port->buffer_num < encoder_output_port->buffer_num_min) {
        encoder_output_port->buffer_num = encoder_output_port->buffer_num_min;
    }

    status = mmal_port_format_commit(encoder_input_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit encoder input port format (%u)\n", status);
        return -1;
    }
    encoder_output_port->format->encoding = MMAL_ENCODING_H264;
    encoder_output_port->format->bitrate = BITRATE;  // ??

/*
    encoder_output_port->buffer_size = encoder_output_port->buffer_size_recommended;

    if (encoder_output_port->buffer_size < encoder_output_port->buffer_size_min) {
        encoder_output_port->buffer_size = encoder_output_port->buffer_size_min;
    }

    encoder_output_port->buffer_num = encoder_output_port->buffer_num_recommended;

    if (encoder_output_port->buffer_num < encoder_output_port->buffer_num_min) {
        encoder_output_port->buffer_num = encoder_output_port->buffer_num_min;
    }
*/
    // Commit the port changes to the output port    
    status = mmal_port_format_commit(encoder_output_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit encoder output port format (%u)\n", status);
        return -1;
    }
    fprintf(stderr, " encoder input buffer_size = %d\n", encoder_input_port->buffer_size);
    fprintf(stderr, " encoder input buffer_num = %d\n", encoder_input_port->buffer_num);

    fprintf(stderr, " encoder output buffer_size = %d\n", encoder_output_port->buffer_size);
    fprintf(stderr, " encoder output buffer_num = %d\n", encoder_output_port->buffer_num);

    encoder_input_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(encoder_input_port, encoder_input_port->buffer_num, encoder_input_port->buffer_size);
    userdata->encoder_input_pool = encoder_input_port_pool;
    encoder_input_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;
    status = mmal_port_enable(encoder_input_port, encoder_input_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder input port (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "INFO:Encoder input pool has been created\n");
    encoder_output_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(encoder_output_port, encoder_output_port->buffer_num, encoder_output_port->buffer_size);
    userdata->encoder_output_pool = encoder_output_port_pool;
    encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    status = mmal_port_enable(encoder_output_port, encoder_output_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder output port (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "INFO:Encoder output pool has been created\n");    

    fill_port_buffer(encoder_output_port, encoder_output_port_pool);

    fprintf(stderr, "INFO:Encoder has been created\n");
    return 0;
}

static void reset_encoder(PORT_USERDATA *userdata) {
    // Get rid of any port buffers first
   mmal_port_disable(userdata->encoder_input_port);
   mmal_port_disable(userdata->encoder_output_port);
   //mmal_component_disable(userdata->encoder);
   if (userdata->encoder_input_pool)
   {
      mmal_port_pool_destroy(userdata->encoder_input_port, userdata->encoder_input_pool);
   }
   if (userdata->encoder_output_pool)
   {
      mmal_port_pool_destroy(userdata->encoder_output_port, userdata->encoder_output_pool);
   }
   if (userdata->encoder)
   {
      mmal_component_destroy(userdata->encoder);
      userdata->encoder = NULL;
   }
}

int setup_preview(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_COMPONENT_T *preview = 0;
    MMAL_CONNECTION_T *camera_preview_connection = 0;
    MMAL_PORT_T *preview_input_port;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &preview);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create preview (%u)\n", status);
        return -1;
    }
    userdata->preview = preview;
    preview_input_port = preview->input[0];

    {
        MMAL_DISPLAYREGION_T param;
        param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
        param.hdr.size = sizeof (MMAL_DISPLAYREGION_T);
        param.set = MMAL_DISPLAY_SET_LAYER;
        param.layer = 0;

        //param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
        // param.fullscreen = 1;
        MMAL_RECT_T previewWindow;
        previewWindow.x = 0;
        previewWindow.y = 0;
        previewWindow.width = VIDEO_WIDTH;
        previewWindow.height = VIDEO_HEIGHT;

        param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
        param.fullscreen = 0;
        param.dest_rect = previewWindow;

        status = mmal_port_parameter_set(preview_input_port, &param.hdr);
        if (status != MMAL_SUCCESS && status != MMAL_ENOSYS) {
            fprintf(stderr, "Error: unable to set preview port parameters (%u)\n", status);
            return -1;
        }
    }


    status = mmal_connection_create(&camera_preview_connection, userdata->camera_preview_port, preview_input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create connection (%u)\n", status);
        return -1;
    }

    status = mmal_connection_enable(camera_preview_connection);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable connection (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "INFO: preview created\n");
    return 0;
}


static void setFilename(char* filename) {
    time_t curTime = time(NULL);
    sprintf(filename, "/tmp/%d.h264", curTime);
}

static void postToQueue(int msqid, char* filename) {
    struct my_msgbuf buf;
    strcpy(buf.mtext, filename);
    buf.mtype = 1;  // sourced from me
    int len = strlen(buf.mtext);

    if (msgsnd(msqid, &buf, len, 0) == -1) 
        perror("msgsnd");
    //
    // Look for something sent to me
    // 
    if (msgrcv(msqid, &buf, sizeof(buf.mtext), 2, IPC_NOWAIT) < 0) {
        switch(errno) {
            case ENOMSG:
                // Cool
                break;
            default:
                perror("msgrcv");
                exit(1);
        }

    } else {
        printf("Client Received: \"%s\"\n", buf.mtext);
        if (strncmp(buf.mtext, "EXIT", 4) == 0) 
            exit(0);
    }
}

int main(int argc, char** argv) {
    MMAL_STATUS_T status;
    PORT_USERDATA userdata;
    int display_width, display_height;
    int msqid;
    key_t key = 500;
    char filename[80];
    char prevFilename[80];

    setFilename(filename);
    userdata.fptr = fopen(filename, "wb");

    printf("Running...\n");

    bcm_host_init();

    if ((msqid = msgget(key, 0644 | IPC_CREAT)) == -1) {
        perror("msgget");
        exit(1);
    }
    if (GX) {
        cvNamedWindow("camcvWin", CV_WINDOW_AUTOSIZE);
    }
    userdata.preview_width = VIDEO_WIDTH / 1;
    userdata.preview_height = VIDEO_HEIGHT / 1;
    userdata.video_width = VIDEO_WIDTH / 1;
    userdata.video_height = VIDEO_HEIGHT / 1;
    userdata.opencv_width = VIDEO_WIDTH / 2;
    userdata.opencv_height = VIDEO_HEIGHT / 2;

    userdata.encoding = MMAL_ENCODING_JPEG;
    userdata.quality = 75;

    graphics_get_display_size(0, &display_width, &display_height);

    printf("Display resolution = (%d, %d)\n", display_width, display_height);

    /* setup opencv */
    userdata.storage = cvCreateMemStorage(0);
    userdata.image1 = cvCreateImage(cvSize(userdata.video_width, userdata.video_height), IPL_DEPTH_8U, 1);
    userdata.image2 = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    userdata.prevImage = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    userdata.py1 = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    userdata.py2 = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);

    if (1 && (status = setup_camera(&userdata) != 0)) {
        fprintf(stderr, "Error: setup camera %x\n", status);
        return -1;
    }
    if (1 && (status = setup_encoder(&userdata) != 0)) {
        fprintf(stderr, "Error: setup encoder %x\n", status);
        return -1;
    }
    if (PREVIEW && (status = setup_preview(&userdata) != 0)) {
        fprintf(stderr, "Error: setup preview %x\n", status);
        return -1;
    }

    vcos_mutex_create(&userdata.filewrite_lock, "snoop_filewrite-lock");
    vcos_semaphore_create(&userdata.complete_semaphore, "snoop_complete-sem", 0);
    vcos_semaphore_create(&userdata.filewrite_semaphore, "snoop_filewrite-sem", 0);
    int opencv_frames = 0;

    GRAPHICS_RESOURCE_HANDLE img_overlay2;
    if (GX) {
        gx_graphics_init("/opt/vc/src/hello_pi/hello_font");
        gx_create_window(0, 900, 400, GRAPHICS_RESOURCE_RGBA32, &img_overlay2);
        graphics_resource_fill(img_overlay2, 0, 0, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, GRAPHICS_RGBA32(0xff, 0, 0, 0x55));
    }

    char text[256];
    int  dataSize = userdata.opencv_width*userdata.opencv_height;
    int  firstFrame = 1;
    int  motionFlag = 0;
    int  pixCount = 0;
    userdata.bufferAction = ACTION_NULL;
    userdata.state = STATE_NORMAL;
    userdata.pendingState = STATE_NORMAL;

    fill_port_buffer(userdata.camera_video_port, userdata.camera_video_port_pool);
    if (mmal_port_parameter_set_boolean(userdata.camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        printf("%s: Failed to start capture\n", __func__);
        exit(-1);
    }
    while (1) {
        if (vcos_semaphore_wait(&(userdata.complete_semaphore)) == VCOS_SUCCESS) {
            switch (userdata.bufferAction) {
                case ACTION_NULL:
                    userdata.image1->imageData = userdata.videoBuffer;  // Hack to avoid memcpy, just copy Y, not UV
                    cvResize(userdata.image1, userdata.image2, CV_INTER_LINEAR);
                    memcpy(userdata.prevImage->imageData, userdata.image2->imageData, dataSize);
                    break;
                case ACTION_CHECK_MOTION:
                    motionFlag = 0;
                    userdata.image1->imageData = userdata.videoBuffer;  // Hack to avoid memcpy, just copy Y, not UV
                    cvResize(userdata.image1, userdata.image2, CV_INTER_LINEAR);
                    if (firstFrame) {
                        firstFrame = 0;
                        memcpy(userdata.prevImage->imageData, userdata.image2->imageData, dataSize);
                    } else {
                        //
                        // Compare images
                        //
                        pixCount = compareImages(&userdata);
                        // cvShowImage("camcvWin", userdata.image1); // display only gray channel
                        // cvWaitKey(1);
                        motionFlag = (pixCount > g_PixelThreshold) ? 1:0;
                        memcpy(userdata.prevImage->imageData, userdata.image2->imageData, dataSize);
                        if (motionFlag) {
                            strcpy(text, "Capture Video");
                            userdata.pendingState = STATE_CAPTURE;
                        }
                    }
                    break;
                case ACTION_STOP_CAPTURE:
                    strcpy(prevFilename, filename); 
                    reset_encoder(&userdata);
                    vcos_mutex_lock(&userdata.filewrite_lock);
                    fflush(userdata.fptr);
                    fclose(userdata.fptr);
                    setFilename(filename);
                    userdata.fptr = fopen(filename, "wb");
                    vcos_mutex_unlock(&userdata.filewrite_lock);
                    strcpy(text, "");
                    userdata.pendingState = STATE_SUSPEND;
                    if (setup_encoder(&userdata) != 0) {
                        fprintf(stderr, "Error: setup encoder %x\n", status);
                        return -1;
                    }
                    postToQueue(msqid, prevFilename);
                    break;
                default:
                    printf("Unknown action: %d\n", userdata.bufferAction);
            }
            if (GX) {
                graphics_resource_fill(img_overlay2, 0, 0, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, GRAPHICS_RGBA32(0, 0, 0, 0x00));
                graphics_resource_render_text_ext(img_overlay2, 0, 0,
                        GRAPHICS_RESOURCE_WIDTH,
                        GRAPHICS_RESOURCE_HEIGHT,
                        GRAPHICS_RGBA32(0x00, 0xff, 0x00, 0xff), /* fg */
                        GRAPHICS_RGBA32(0, 0, 0, 0x00), /* bg */
                        text, strlen(text), 25);
                graphics_display_resource(img_overlay2, 0, 2, 0, display_width / 16, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, VC_DISPMAN_ROT0, 1);
            }
        }
    }
    return 0;
}

