/**
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup zigbee_examples_thingy_master_zed_color_light_bulb zigbee_color_light.h
 * @{
 * @ingroup zigbee_examples
 * @brief Dynamic multiprotocol example application to demonstrate control on BLE device (peripheral role) using zigbee device.
 */

#ifndef ZIGBEE_COLOR_LIGHT_H__
#define ZIGBEE_COLOR_LIGHT_H__

#include <stdint.h>
#include "zigbee_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ZigBee device configuration values. */
#define IEEE_CHANNEL_MASK                       (1l << ZIGBEE_CHANNEL)              /**< Scan only one, predefined channel to find the coordinator. */
#define BULB_LOCATION_KITCHEN                   0x1D
#define BULB_LOCATION_OFFICE                    0x24

/* Basic cluster attributes initial values. */
#define BULB_INIT_BASIC_APP_VERSION             01                                  /**< Version of the application software (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION           10                                  /**< Version of the implementation of the ZigBee stack (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION              11                                  /**< Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_MANUF_NAME              "Nordic"                            /**< Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MODEL_ID                "Color_Dimable_Light_Bulb_v0.1"     /**< Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_DATE_CODE               "20180718"                          /**< First 8 bytes specify the date of manufacturer of the device in ISO 8601 format (YYYYMMDD). Th rest (8 bytes) are manufacturer specific. */
#define BULB_INIT_BASIC_POWER_SOURCE            ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE /**< Type of power sources available for the device. For possible values see section 3.2.2.2.8 of ZCL specification. */
#define BULB_INIT_BASIC_LOCATION_DESC           "Office desk"                       /**< Describes the physical location of the device (16 bytes). May be modified during commisioning process. */
#define BULB_INIT_BASIC_PH_ENV                  BULB_LOCATION_OFFICE                /**< Describes the type of physical environment. For possible values see section 3.2.2.2.10 of ZCL specification. */

#define ZB_HA_DEVICE_VER_COLOR_CONTROL          0                                   /**< Color light device version. */
#define ZB_HA_COLOR_CONTROL_IN_CLUSTER_NUM      8                                   /**< Color light input clusters number. */
#define ZB_HA_COLOR_CONTROL_OUT_CLUSTER_NUM     0                                   /**< Color light output clusters number. */

#define ZB_ZCL_COLOR_DIMMABLE_LIGHT_CVC_ATTR_COUNT (ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT + 3)

/**@brief Redefinition of __CAT__ as variadic macro
 * 
 * @param[IN]  a   Mandatory argument to concatenate
 * @param[IN]  b   Mandatory argument to concatenate
 * @param[IN]  ... Optional argument to concatenate
 */
#define __CAT_VA__(a, b, ...)                               a## __VA_ARGS__## b

/**@brief Redefinition of ZB_AF_SIMPLE_DESC_TYPE as variadic macro
 *
 * @param[IN]  ...      Optional argument to concatenate to type name
 */
#define ZB_AF_SIMPLE_DESC_TYPE_VA(in_num, out_num, ...)     __CAT_VA__(zb_af_simple_desc_, _t, __VA_ARGS__)

/**@brief Redefinition of ZB_DECLARE_SIMPLE_DESC as variadic macro
 * 
 * @param[IN]  in_clusters_count   Number of input clusters
 * @param[IN]  out_clusters_count  Number of output clusters
 * @param[IN]  ...                 Optional argument to concatenate to type name
 */
#define ZB_DECLARE_SIMPLE_DESC_VA(in_clusters_count, out_clusters_count, ...)                                   \
    typedef ZB_PACKED_PRE struct zb_af_simple_desc_## __VA_ARGS__## _s                                          \
    {                                                                                                           \
        zb_uint8_t    endpoint;                                         /* Endpoint */                          \
        zb_uint16_t   app_profile_id;                                   /* Application profile identifier */    \
        zb_uint16_t   app_device_id;                                    /* Application device identifier */     \
        zb_bitfield_t app_device_version:4;                             /* Application device version */        \
        zb_bitfield_t reserved:4;                                       /* Reserved */                          \
        zb_uint8_t    app_input_cluster_count;                          /* Application input cluster count */   \
        zb_uint8_t    app_output_cluster_count;                         /* Application output cluster count */  \
        /* Application input and output cluster list */                                                         \
        zb_uint16_t   app_cluster_list[in_clusters_count + out_clusters_count];                                 \
    } ZB_PACKED_STRUCT zb_af_simple_desc_## __VA_ARGS__## _t

/**@brief Declare color light simple descriptor.
 * 
 * @param[IN] ep_name                endpoint variable name.
 * @param[IN] ep_id [IN]             endpoint ID.
 * @param[IN] in_clust_num           number of supported input clusters.
 * @param[IN] out_clust_num          number of supported output clusters.
 * 
 * @note in_clust_num, out_clust_num should be defined by numeric constants, not variables or any
 * definitions, because these values are used to form simple descriptor type name.
 */
#define ZB_ZCL_DECLARE_COLOR_LIGHT_SIMPLE_DESC(ep_name, ep_id, in_clust_num, out_clust_num)             \
            ZB_DECLARE_SIMPLE_DESC_VA(in_clust_num, out_clust_num, ep_name);                            \
            ZB_AF_SIMPLE_DESC_TYPE_VA(in_clust_num, out_clust_num, ep_name)  simple_desc_## ep_name =   \
            {                                                                                           \
                ep_id,                                                                                  \
                ZB_AF_HA_PROFILE_ID,                                                                    \
                ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,                                                   \
                ZB_HA_DEVICE_VER_COLOR_CONTROL,                                                         \
                0,                                                                                      \
                in_clust_num,                                                                           \
                out_clust_num,                                                                          \
                {                                                                                       \
                  ZB_ZCL_CLUSTER_ID_BASIC,                                                              \
                  ZB_ZCL_CLUSTER_ID_IDENTIFY,                                                           \
                  ZB_ZCL_CLUSTER_ID_GROUPS,                                                             \
                  ZB_ZCL_CLUSTER_ID_SCENES,                                                             \
                  ZB_ZCL_CLUSTER_ID_ON_OFF,                                                             \
                  ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,                                                      \
                  ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,                                                      \
                  ZB_ZCL_CLUSTER_ID_COMMISSIONING                                                       \
                }                                                                                       \
            }

/**@brief Declare Color Light endpoint.
 * 
 * @param[IN] ep_name                endpoint variable name.
 * @param[IN] ep_id [IN]             endpoint ID.
 * @param[IN] cluster_list [IN]      list of endpoint clusters
 */
#define ZB_ZCL_DECLARE_COLOR_LIGHT_EP(ep_name, ep_id, cluster_list)                 \
    ZB_ZCL_DECLARE_COLOR_LIGHT_SIMPLE_DESC(                                         \
        ep_name,                                                                    \
        ep_id,                                                                      \
        ZB_HA_COLOR_CONTROL_IN_CLUSTER_NUM,                                         \
        ZB_HA_COLOR_CONTROL_OUT_CLUSTER_NUM);                                       \
    ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info## ep_name,                    \
        (ZB_ZCL_COLOR_CONTROL_REPORT_ATTR_COUNT));                                  \
    ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info## ep_name,                \
        ZB_ZCL_COLOR_DIMMABLE_LIGHT_CVC_ATTR_COUNT);                                \
    ZB_AF_DECLARE_ENDPOINT_DESC(                                                    \
        ep_name,                                                                    \
        ep_id,                                                                      \
        ZB_AF_HA_PROFILE_ID,                                                        \
        0,                                                                          \
        NULL                                       ,                                \
        ZB_ZCL_ARRAY_SIZE(                                                          \
            cluster_list,                                                           \
            zb_zcl_cluster_desc_t),                                                 \
        cluster_list,                                                               \
        (zb_af_simple_desc_1_1_t*)&simple_desc_## ep_name,                          \
        ZB_ZCL_COLOR_CONTROL_REPORT_ATTR_COUNT,                                     \
        reporting_info## ep_name,                                                   \
        ZB_ZCL_COLOR_DIMMABLE_LIGHT_CVC_ATTR_COUNT,                                 \
        cvc_alarm_info## ep_name)

/**@brief Declare cluster list for Color light device.
 * 
 * @param[IN] cluster_list_name [IN]             cluster list variable name.
 * @param[IN] basic_attr_list [IN]               attribute list for Basic cluster.
 * @param[IN] identify_attr_list [IN]            attribute list for Identify cluster.
 * @param[IN] groups_attr_list [IN]              attribute list for Groups cluster.
 * @param[IN] scenes_attr_list [IN]              attribute list for Scenes cluster.
 * @param[IN] on_off_attr_list [IN]              attribute list for On/Off cluster.
 * @param[IN] level_control_attr_list [IN]       attribute list for Level Control cluster.
 * @param[IN] color_control_attr_list [IN]       attribute list for Color Control cluster.
 * @param[IN] commissioning_declare_both [IN]    determines Commissioning cluster role: ZB_TRUE implies
 * both client and server, and ZB_FALSE implies server role only.
 */
#define ZB_ZCL_DECLARE_COLOR_LIGHT_CLUSTER_LIST(                                                \
    cluster_list_name,                                                                          \
    basic_attr_list,                                                                            \
    identify_attr_list,                                                                         \
    groups_attr_list,                                                                           \
    scenes_attr_list,                                                                           \
    on_off_attr_list,                                                                           \
    level_control_attr_list,                                                                    \
    color_control_attr_list,                                                                    \
    commissioning_declare_both)                                                                 \
    zb_zcl_cluster_desc_t cluster_list_name[] =                                                 \
    {                                                                                           \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_BASIC,                                                                \
        ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),                                      \
        (basic_attr_list),                                                                      \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        ),                                                                                      \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_IDENTIFY,                                                             \
        ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t),                                   \
        (identify_attr_list),                                                                   \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        ),                                                                                      \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_GROUPS,                                                               \
        ZB_ZCL_ARRAY_SIZE(groups_attr_list, zb_zcl_attr_t),                                     \
        (groups_attr_list),                                                                     \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        ),                                                                                      \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_SCENES,                                                               \
        ZB_ZCL_ARRAY_SIZE(scenes_attr_list, zb_zcl_attr_t),                                     \
        (scenes_attr_list),                                                                     \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        ),                                                                                      \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_ON_OFF,                                                               \
        ZB_ZCL_ARRAY_SIZE(on_off_attr_list, zb_zcl_attr_t),                                     \
        (on_off_attr_list),                                                                     \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        ),                                                                                      \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,                                                        \
        ZB_ZCL_ARRAY_SIZE(level_control_attr_list, zb_zcl_attr_t),                              \
        (level_control_attr_list),                                                              \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        ),                                                                                      \
        ZB_ZCL_CLUSTER_DESC(                                                                    \
        ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,                                                        \
        ZB_ZCL_ARRAY_SIZE(color_control_attr_list, zb_zcl_attr_t),                              \
        (color_control_attr_list),                                                              \
        ZB_ZCL_CLUSTER_SERVER_ROLE,                                                             \
        ZB_ZCL_MANUF_CODE_INVALID                                                               \
        )                                                                                       \
    }

/**@brief Declare attribute list for Level Control cluster, defined as variadic macro
 * 
 * @param[IN] attr_list          attribure list name
 * @param[IN] current_level      pointer to variable to store current_level attribute value
 * @param[IN] remaining_time     pointer to variable to store remaining_time attribute value
 * @param[IN] ...                Optional argument to concatenate to variable name
 */
#define ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST_VA(attr_list, current_level, remaining_time, ...)  \
  zb_zcl_level_control_move_status_t move_status_data_ctx## __VA_ARGS__## _attr_list ;              \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                                       \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, (current_level))                 \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_REMAINING_TIME_ID, (remaining_time))               \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_MOVE_STATUS_ID,                                    \
                       (&(move_status_data_ctx## __VA_ARGS__## _attr_list)))                        \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

/**@brief Declare color light bulb cluster attribute list
 * 
 * @param[IN] dev_ctx_name                       name of variable to store device contex in.
 * @param[IN] color_light_bulb_cluster_list      List of clusters name.
 */
#define ZB_DECLARE_COLOR_LIGHT_BULB_CLUSTER_ATTR_LIST(dev_ctx_name, color_light_bulb_cluster_list)                                         \
    ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST_HA(dev_ctx_name## _identify_attr_list,                                                             \
                                           &dev_ctx_name.identify_attr.identify_time,                                                      \
                                           &dev_ctx_name.identify_attr.commission_state);                                                  \
    ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(dev_ctx_name## _groups_attr_list, &dev_ctx_name.groups_attr.name_support);                           \
    ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(dev_ctx_name## _scenes_attr_list,                                                                    \
                                      &dev_ctx_name.scenes_attr.scene_count,                                                               \
                                      &dev_ctx_name.scenes_attr.current_scene,                                                             \
                                      &dev_ctx_name.scenes_attr.current_group,                                                             \
                                      &dev_ctx_name.scenes_attr.scene_valid,                                                               \
                                      &dev_ctx_name.scenes_attr.name_support);                                                             \
    ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_HA_ADDS_FULL(dev_ctx_name## _basic_attr_list,                                                         \
                                                  &dev_ctx_name.basic_attr.zcl_version,                                                    \
                                                  &dev_ctx_name.basic_attr.app_version,                                                    \
                                                  &dev_ctx_name.basic_attr.stack_version,                                                  \
                                                  &dev_ctx_name.basic_attr.hw_version,                                                     \
                                                  dev_ctx_name.basic_attr.mf_name,                                                         \
                                                  dev_ctx_name.basic_attr.model_id,                                                        \
                                                  dev_ctx_name.basic_attr.date_code,                                                       \
                                                  &dev_ctx_name.basic_attr.power_source,                                                   \
                                                  dev_ctx_name.basic_attr.location_id,                                                     \
                                                  &dev_ctx_name.basic_attr.ph_env);                                                        \
    ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST_TL(dev_ctx_name## _on_off_attr_list,                                                                 \
                                          &dev_ctx_name.on_off_attr.on_off,                                                                \
                                          &dev_ctx_name.on_off_attr.global_scene_ctrl,                                                     \
                                          &dev_ctx_name.on_off_attr.on_time,                                                               \
                                          &dev_ctx_name.on_off_attr.off_wait_time);                                                        \
    ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST_VA(dev_ctx_name## _level_control_attr_list,                                                   \
                                              &dev_ctx_name.level_control_attr.current_level,                                              \
                                              &dev_ctx_name.level_control_attr.remaining_time,                                             \
                                              dev_ctx_name);                                                                               \
    ZB_ZCL_DECLARE_COLOR_CONTROL_ATTRIB_LIST(dev_ctx_name## _color_control_attr_list,                                                      \
                                           &dev_ctx_name.color_control_attr.set_color_info.current_hue,                                    \
                                           &dev_ctx_name.color_control_attr.set_color_info.current_saturation,                             \
                                           &dev_ctx_name.color_control_attr.set_color_info.remaining_time,                                 \
                                           &dev_ctx_name.color_control_attr.set_color_info.current_X,                                      \
                                           &dev_ctx_name.color_control_attr.set_color_info.current_Y,                                      \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_temperature,                              \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_mode,                                     \
                                           &dev_ctx_name.color_control_attr.set_color_info.options,                                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.number_primaries,                   \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_1_X,                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_1_Y,                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_1_intensity,                \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_2_X,                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_2_Y,                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_2_intensity,                \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_3_X,                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_3_Y,                        \
                                           &dev_ctx_name.color_control_attr.set_defined_primaries_info.primary_3_intensity,                \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_4_X,             \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_4_Y,             \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_4_intensity,     \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_5_X,             \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_5_Y,             \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_5_intensity,     \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_6_X,             \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_6_Y,             \
                                           &dev_ctx_name.color_control_attr.set_additional_defined_primaries_info.primary_6_intensity,     \
                                           &dev_ctx_name.color_control_attr.set_color_info.enhanced_current_hue,                           \
                                           &dev_ctx_name.color_control_attr.set_color_info.enhanced_color_mode,                            \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_loop_active,                              \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_loop_direction,                           \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_loop_time,                                \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_loop_start_enhanced_hue,                  \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_loop_stored_enhanced_hue,                 \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_capabilities,                             \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_temp_physical_min_mireds,                 \
                                           &dev_ctx_name.color_control_attr.set_color_info.color_temp_physical_max_mireds,                 \
                                           &dev_ctx_name.color_control_attr.set_color_info.couple_color_temp_to_level_min_mireds,          \
                                           &dev_ctx_name.color_control_attr.set_color_info.start_up_color_temp_mireds);                    \
    ZB_ZCL_DECLARE_COLOR_LIGHT_CLUSTER_LIST(color_light_bulb_cluster_list,                                                                 \
                                            dev_ctx_name## _basic_attr_list,                                                               \
                                            dev_ctx_name## _identify_attr_list,                                                            \
                                            dev_ctx_name## _groups_attr_list,                                                              \
                                            dev_ctx_name## _scenes_attr_list,                                                              \
                                            dev_ctx_name## _on_off_attr_list,                                                              \
                                            dev_ctx_name## _level_control_attr_list,                                                       \
                                            dev_ctx_name## _color_control_attr_list,                                                       \
                                            ZB_FALSE);

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile light bulb (end-device) source code.
#endif

/* Zigbee color light bulb cluster context. Stores all settings and static values. */
typedef struct
{
    zb_dev_basic_attr_t            basic_attr;
    zb_dev_identify_attr_t         identify_attr;
    zb_dev_scenes_attr_t           scenes_attr;
    zb_dev_groups_attr_t           groups_attr;
    zb_dev_on_off_attr_t           on_off_attr;
    zb_dev_level_control_attr_t    level_control_attr;
    zb_dev_color_control_attr_t    color_control_attr;
} zb_bulb_dev_ctx_t;

#ifdef __cplusplus
}
#endif
#endif // ZIGBEE_COLOR_LIGHT_H__


/**
 * @}
 */
