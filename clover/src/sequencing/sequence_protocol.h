#pragma once
#include "clover.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Same Request/Response and length-delimited framing as Clover's TCP server. */
void sequence_handle_request(const Request *request, Response *response);

#ifdef __cplusplus
}
#endif
