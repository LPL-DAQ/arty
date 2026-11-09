#pragma once
#include "clover.pb.h"

/* Same Request/Response and length-delimited framing as Clover's TCP server. */
void sequence_handle_request(const Request *request, Response *response);
