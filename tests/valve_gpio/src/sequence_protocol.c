#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "sequence_engine.h"
#include "sequence_protocol.h"

/* Shared adapter from wire operations to the same engine used by the USB shell. */
void sequence_handle_request(const Request *request, Response *response)
{
    /* Responses are reused by the server; clear optional-field flags on every call. */
    *response = (Response)Response_init_zero;
    bool manual = request->which_payload == Request_manual_valve_tag;
    bool abort = request->which_payload == Request_abort_tag;
    if (!abort && !manual && request->which_payload != Request_named_sequence_tag) {
        response->has_err = true;
        snprintf(response->err, sizeof(response->err),
                 "Standalone valve test supports named_sequence, manual_valve and abort only");
        return;
    }
    const NamedSequenceRequest *command = &request->payload.named_sequence;
    int err = 0;
    if (manual) {
        err = sequence_manual_valve(request->payload.manual_valve.channel,
                                    request->payload.manual_valve.powered);
    } else if (abort) {
        err = sequence_abort();
    } else switch (command->operation) {
    case NamedSequenceRequest_Operation_LIST:
    case NamedSequenceRequest_Operation_STATUS:
        break;
    case NamedSequenceRequest_Operation_RUN:
        err = command->has_name && command->name[0] ? sequence_start(command->name) : -EINVAL;
        break;
    case NamedSequenceRequest_Operation_STOP:
        err = sequence_abort(); /* Legacy wire operation; new clients send AbortRequest. */
        break;
    default:
        err = -EINVAL;
        break;
    }
    if (err) {
        response->has_err = true;
        snprintf(response->err, sizeof(response->err),
                 "Valve/sequence request rejected (%d): check channel, name and status; faults require reset", err);
    }
    /* Report a coherent engine snapshot after command acceptance, not run completion. */
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
    if (strcmp(status.state, "running") && strcmp(status.state, "aborting") &&
        strcmp(status.state, "initializing") && strcmp(status.state, "faulted")) {
        info->has_manual_powered_channels = true;
        info->manual_powered_channels = status.manual_powered_channels;
    }
    if (!abort && !manual && command->operation == NamedSequenceRequest_Operation_LIST) {
        for (size_t i = 0; i < sequence_count(); ++i) {
            snprintf(info->names[info->names_count++], sizeof(info->names[0]), "%s", sequence_name(i));
        }
    }
}
