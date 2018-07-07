#ifndef __AP6211_H__
#define __AP6211_H__

#define AP6211_EMERG(...)	pr_emerg("[ap6211] "__VA_ARGS__)
#define AP6211_ALERT(...)	pr_alert("[ap6211] "__VA_ARGS__)
#define AP6211_CRIT(...)	pr_crit("[ap6211] "__VA_ARGS__)
#define AP6211_ERR(...)		pr_err("[ap6211] "__VA_ARGS__)
#define AP6211_WARN(...)	pr_warn("[ap6211] "__VA_ARGS__)
#define AP6211_NOTICE(...)	pr_notice("[ap6211] "__VA_ARGS__)
#define AP6211_INFO(...)	pr_info("[ap6211] "__VA_ARGS__)
#define AP6211_DEBUG(...)	pr_debug("[ap6211] "__VA_ARGS__)
#define AP6211_DUMP(...)	pr_debug(__VA_ARGS__)
#define AP6211_CONT(...)	pr_cont(__VA_ARGS__)

extern int __init sw_rfkill_init(void);
extern void __exit sw_rfkill_exit(void);

extern int __init ap6211_gpio_wifi_init(void);
extern void __exit ap6211_gpio_wifi_exit(void);


#endif  /* __AP6211_H__ */
