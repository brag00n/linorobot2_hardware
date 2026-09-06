/*
 * stm32f10x_conf.h - version MINIMALE pour le bootloader IAP.
 *
 * stm32f10x.h fait "#include \"stm32f10x_conf.h\"" quand USE_STDPERIPH_DRIVER
 * est defini (ce qui est le cas par defaut). Le bootloader n'utilise PAS la
 * StdPeriph (FWlib) : il pilote les registres directement. On fournit donc ce
 * conf.h vide (place sur SON include path, avant celui du projet applicatif)
 * pour ne tirer aucun header FWlib et garder l'image sous 16 KB.
 */
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* assert_param : no-op (aucune source FWlib n'est compilee ici). */
#define assert_param(expr) ((void)0)

#endif /* __STM32F10x_CONF_H */
