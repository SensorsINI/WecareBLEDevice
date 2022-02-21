#ifndef MY_OWN_APP_DISS_TASK_H_
#define MY_OWN_APP_DISS_TASK_H_

#include "rwip_config.h"     // SW configuration
#include "ke_task.h"
#include "diss_task.h"          // Device Information Service Server Task API

int my_own_diss_value_req_ind_handler(ke_msg_id_t const msgid,
                                      struct diss_value_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);
																			
#endif
																			