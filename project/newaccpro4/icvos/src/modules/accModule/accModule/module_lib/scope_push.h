#ifndef SCOPE_PUSH_H
#define SCOPE_PUSH_H

/*
 * Generated scope data bridge. Records are appended to:
 *   ${GAASD_SCOPE_PUSH_DIR:-/tmp/gaasd_scope_push}/<scope_id>.jsonl
 */

#ifdef __cplusplus
extern "C" {
#endif

extern char scope_push_list_end_tag;
#define SCOPE_PUSH_LIST_END ((const char*)&scope_push_list_end_tag)

void scope_push_send_v(const char* scope_id, ...);
#define scope_push_send(scope_id, ...) \
    scope_push_send_v((scope_id), ##__VA_ARGS__, SCOPE_PUSH_LIST_END)

#ifdef __cplusplus
}
#endif

#endif
