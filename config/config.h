// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef USE_BAMBOOV200_CONFIG
    #include "custom/bamboov200_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_VATTENKAR_CONFIG
    #include "custom/vattenkar_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_DEV_CONFIG
    #include "custom/dev_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_GENDRV_CONFIG
    #include "custom/gendrv_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_GENDRV_WIFI_CONFIG
    #include "custom/gendrv_wifi_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_ESP32_CONFIG
    #include "custom/esp32_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_ESP32_WIFI_CONFIG
    #include "custom/esp32_wifi_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_ESP32S2_CONFIG
    #include "custom/esp32s2_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_ESP32S2_WIFI_CONFIG
    #include "custom/esp32s2_wifi_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_ESP32S3_CONFIG
    #include "custom/esp32s3_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_ESP32S3_WIFI_CONFIG
    #include "custom/esp32s3_wifi_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_PICO2_CONFIG
    #include "custom/pico2_config.h"
    #define LINO_CONFIG
#endif

#ifdef USE_PICO_CONFIG
    #include "custom/pico_config.h"
    #define LINO_CONFIG
#endif


#if !defined (LINO_CONFIG) 
    #include "lino_base_config.h"
#endif

