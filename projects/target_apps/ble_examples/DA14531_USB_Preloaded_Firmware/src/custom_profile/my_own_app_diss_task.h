#ifndef MY_OWN_APP_DISS_TASK_H_
#define MY_OWN_APP_DISS_TASK_H_

int diss_value_req_ind_handler(ke_msg_id_t const msgid,
                                      struct diss_value_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);
#endif