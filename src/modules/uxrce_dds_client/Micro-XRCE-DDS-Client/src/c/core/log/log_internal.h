/* Debug stub - enables compilation of session.c debug code */
#ifndef UXR_DEBUG_PRINT_MESSAGE
#define UXR_DEBUG_PRINT_MESSAGE(op, buf, len, key) ((void)0)
#endif

/* Debug operation types used in send_message/recv_message */
#ifndef UXR_SEND
#define UXR_SEND 0
#endif
#ifndef UXR_ERROR_SEND
#define UXR_ERROR_SEND 1
#endif
#ifndef UXR_RECV
#define UXR_RECV 2
#endif
