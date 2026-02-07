/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026
 *
 * Author: Yuzhii
 *
 * Internal interfaces for Failsafe Web UI modules
 */

#ifndef _FAILSAFE_INTERNAL_H_
#define _FAILSAFE_INTERNAL_H_

#include <net/mtk_httpd.h>
#include <linux/types.h>

size_t json_escape(char *dst, size_t dst_sz, const char *src);

#ifdef CONFIG_WEBUI_FAILSAFE_CONSOLE
int failsafe_webconsole_ensure_recording(void);
void webconsole_poll_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void webconsole_exec_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void webconsole_clear_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
#endif

#ifdef CONFIG_WEBUI_FAILSAFE_ENV
void env_list_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void env_set_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void env_unset_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void env_reset_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void env_restore_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
#endif

#ifdef CONFIG_WEBUI_FAILSAFE_BACKUP
void backupinfo_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
void backup_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
#endif

#ifdef CONFIG_WEBUI_FAILSAFE_FLASH
void flash_handler(enum httpd_uri_handler_status status,
	struct httpd_request *request,
	struct httpd_response *response);
#endif

#endif /* _FAILSAFE_INTERNAL_H_ */
