#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "sequence_engine.h"
#include "sequence_protocol.h"

void sequence_handle_request(const Request *request, Response *response)
{
    *response = (Response)Response_init_zero;
    if (request->which_payload != Request_named_sequence_tag) {
        response->has_err = true;
        snprintf(response->err, sizeof(response->err),
                 "Standalone valve test supports named_sequence only; use Clover for production commands");
        return;
    }
    const NamedSequenceRequest *command = &request->payload.named_sequence;
    int err = 0;
    switch (command->operation) {
    case NamedSequenceRequest_Operation_LIST:
    case NamedSequenceRequest_Operation_STATUS:
        break;
    case NamedSequenceRequest_Operation_RUN:
        err = command->has_name && command->name[0] ? sequence_start(command->name) : -EINVAL;
        break;
    case NamedSequenceRequest_Operation_STOP:
        err = sequence_stop();
        break;
    default:
        err = -EINVAL;
        break;
    }
    if (err) {
        response->has_err = true;
        snprintf(response->err, sizeof(response->err),
                 "Sequence request rejected (%d): check name and status; faults require reset", err);
    }
    struct sequence_snapshot status = sequence_status();
    response->has_named_sequence = true;
    NamedSequenceInfo *info = &response->named_sequence;
    snprintf(info->state, sizeof(info->state), "%s", status.state);
    info->has_name = true;
    snprintf(info->name, sizeof(info->name), "%s", status.name);
    info->has_comment = true;
    snprintf(info->comment, sizeof(info->comment), "%s", status.comment);
    info->has_step = info->has_total_steps = info->has_error = true;
    info->step = status.step;
    info->total_steps = status.total_steps;
    info->error = status.error;
    if (command->operation == NamedSequenceRequest_Operation_LIST) {
        for (size_t i = 0; i < sequence_count(); ++i) {
            snprintf(info->names[info->names_count++], sizeof(info->names[0]), "%s", sequence_name(i));
        }
    }
}
