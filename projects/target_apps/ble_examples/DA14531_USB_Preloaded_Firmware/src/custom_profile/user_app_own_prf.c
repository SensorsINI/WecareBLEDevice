#include "rwip_config.h"     // SW configuration


#include "user_app_own_prf.h"                 
#include "app.h"                     // Application Definitions
#include "app_task.h"                // Application Task Definitions
#include "app_prf_perm_types.h"
#include "user_profiles_config.h"

#include "da14531_printf.h"

void app_own_prf_creat_db(void)
{
    struct diss_db_cfg* db_cfg;
    
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM, 
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd, 
                                                             sizeof(struct diss_db_cfg));
    
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = get_user_prf_srv_perm(88);
    req->prf_task_id = 88;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct diss_db_cfg* ) req->param;
    db_cfg->features = APP_DIS_FEATURES;

    // Send the message
    ke_msg_send(req);
}
