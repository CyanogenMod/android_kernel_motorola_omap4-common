
/* Functions called by the TI PPA framework */
void	mot_ppa_init(U32 ResetReason);
U32	mot_ppa_hal_dispatcher(U32 id);

/* Motorola PPA services
   Note service IDs must start somewhere after __ppa_hal_services_end */
#define API_HAL_MOT_EFUSE_WRITE		1000
#define API_HAL_MOT_EFUSE_READ		1001
#define API_HAL_MOT_EFUSE_LOCK		1002
#define API_HAL_MOT_GET_DIE_TEMP	1003
#define API_HAL_MOT_SHA256_Digest	1004

#define API_HAL_KM_PUBLIC_ID			1100
#define API_HAL_KM_CERTIFICATE_SIGNATURE_VERIFY 1101
