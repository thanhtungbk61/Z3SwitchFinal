// This file is generated by Simplicity Studio.  Please do not edit manually.
//
//

// This is a set of generated functions that parse the
// the incomming message, and call appropriate command handler.



#include PLATFORM_HEADER
#ifdef EZSP_HOST
// Includes needed for ember related functions for the EZSP host
#include "stack/include/error.h"
#include "stack/include/ember-types.h"
#include "app/util/ezsp/ezsp-protocol.h"
#include "app/util/ezsp/ezsp.h"
#include "app/util/ezsp/ezsp-utils.h"
#include "app/util/ezsp/serial-interface.h"
#else
// Includes needed for ember related functions for the EM250
#include "stack/include/ember.h"
#endif // EZSP_HOST

#include "app/framework/util/util.h"
#include "af-structs.h"
#include "call-command-handler.h"
#include "command-id.h"
#include "callback.h"

static EmberAfStatus status(bool wasHandled, bool clusterExists, bool mfgSpecific)
{
  if (wasHandled) {
    return EMBER_ZCL_STATUS_SUCCESS;
  } else if (mfgSpecific) {
    return EMBER_ZCL_STATUS_UNSUP_MANUF_CLUSTER_COMMAND;
  } else if (clusterExists) {
    return EMBER_ZCL_STATUS_UNSUP_CLUSTER_COMMAND;
  } else {
    return EMBER_ZCL_STATUS_UNSUPPORTED_CLUSTER;
  }
}

// Main command parsing controller.
EmberAfStatus emberAfClusterSpecificCommandParse(EmberAfClusterCommand *cmd)
{
  EmberAfStatus result = status(false, false, cmd->mfgSpecific);
  if (cmd->direction == (uint8_t)ZCL_DIRECTION_SERVER_TO_CLIENT
      && emberAfContainsClientWithMfgCode(cmd->apsFrame->destinationEndpoint,
                               cmd->apsFrame->clusterId,
                               cmd->mfgCode)) {
    switch (cmd->apsFrame->clusterId) {
    case ZCL_IDENTIFY_CLUSTER_ID:
      result = emberAfIdentifyClusterClientCommandParse(cmd);
      break;
    case ZCL_GROUPS_CLUSTER_ID:
      result = emberAfGroupsClusterClientCommandParse(cmd);
      break;
    case ZCL_SCENES_CLUSTER_ID:
      result = emberAfScenesClusterClientCommandParse(cmd);
      break;
    case ZCL_ON_OFF_CLUSTER_ID:
      result = status(false, true, cmd->mfgSpecific);
      break;
    case ZCL_LEVEL_CONTROL_CLUSTER_ID:
      result = status(false, true, cmd->mfgSpecific);
      break;
    case ZCL_COLOR_CONTROL_CLUSTER_ID:
      result = status(false, true, cmd->mfgSpecific);
      break;
    case ZCL_ILLUM_MEASUREMENT_CLUSTER_ID:
      result = status(false, true, cmd->mfgSpecific);
      break;
    case ZCL_ZLL_COMMISSIONING_CLUSTER_ID:
      result = emberAfZllCommissioningClusterClientCommandParse(cmd);
      break;
    case ZCL_MANAGER_ID:
      result = status(false, true, cmd->mfgSpecific);
      break;
    default:
      // Unrecognized cluster ID, error status will apply.
      break;
    }
  } else if (cmd->direction == (uint8_t)ZCL_DIRECTION_CLIENT_TO_SERVER
             && emberAfContainsServerWithMfgCode(cmd->apsFrame->destinationEndpoint,
                                       cmd->apsFrame->clusterId,
                                       cmd->mfgCode)) {
    switch (cmd->apsFrame->clusterId) {
    case ZCL_BASIC_CLUSTER_ID:
      result = emberAfBasicClusterServerCommandParse(cmd);
      break;
    case ZCL_IDENTIFY_CLUSTER_ID:
      result = emberAfIdentifyClusterServerCommandParse(cmd);
      break;
    case ZCL_OTA_BOOTLOAD_CLUSTER_ID:
      result = status(false, true, cmd->mfgSpecific);
      break;
    case ZCL_MANAGER_ID:
      result = emberAfManagerServerCommandParse(cmd);
      break;
    default:
      // Unrecognized cluster ID, error status will apply.
      break;
    }
  }
  return result;
}

// Cluster: Basic, server
EmberAfStatus emberAfBasicClusterServerCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (!cmd->mfgSpecific) {
    switch (cmd->commandId) {
    case ZCL_RESET_TO_FACTORY_DEFAULTS_COMMAND_ID:
      {
        // Command is fixed length: 0
        wasHandled = emberAfBasicClusterResetToFactoryDefaultsCallback();
        break;
      }
    default:
      {
        // Unrecognized command ID, error status will apply.
        break;
      }
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}

// Cluster: Identify, client
EmberAfStatus emberAfIdentifyClusterClientCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (!cmd->mfgSpecific) {
    switch (cmd->commandId) {
    case ZCL_IDENTIFY_QUERY_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint16_t timeout;  // Ver.: always
        // Command is fixed length: 2
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        timeout = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfIdentifyClusterIdentifyQueryResponseCallback(timeout);
        break;
      }
    default:
      {
        // Unrecognized command ID, error status will apply.
        break;
      }
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}

// Cluster: Identify, server
EmberAfStatus emberAfIdentifyClusterServerCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (!cmd->mfgSpecific) {
    switch (cmd->commandId) {
    case ZCL_IDENTIFY_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint16_t identifyTime;  // Ver.: always
        // Command is fixed length: 2
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        identifyTime = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfIdentifyClusterIdentifyCallback(identifyTime);
        break;
      }
    case ZCL_IDENTIFY_QUERY_COMMAND_ID:
      {
        // Command is fixed length: 0
        wasHandled = emberAfIdentifyClusterIdentifyQueryCallback();
        break;
      }
    default:
      {
        // Unrecognized command ID, error status will apply.
        break;
      }
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}

// Cluster: Groups, client
EmberAfStatus emberAfGroupsClusterClientCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (!cmd->mfgSpecific) {
    switch (cmd->commandId) {
    case ZCL_ADD_GROUP_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        // Command is fixed length: 3
        if (cmd->bufLen < payloadOffset + 3u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfGroupsClusterAddGroupResponseCallback(status,
                                                                  groupId);
        break;
      }
    case ZCL_VIEW_GROUP_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t* groupName;  // Ver.: always
        // Command is not a fixed length
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        if (cmd->bufLen < payloadOffset + emberAfStringLength(cmd->buffer + payloadOffset) + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        groupName = emberAfGetString(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfGroupsClusterViewGroupResponseCallback(status,
                                                                   groupId,
                                                                   groupName);
        break;
      }
    case ZCL_GET_GROUP_MEMBERSHIP_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t capacity;  // Ver.: always
        uint8_t groupCount;  // Ver.: always
        uint8_t* groupList;  // Ver.: always
        // Command is fixed length: 2
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        capacity = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupCount = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupList = cmd->buffer + payloadOffset;
        wasHandled = emberAfGroupsClusterGetGroupMembershipResponseCallback(capacity,
                                                                            groupCount,
                                                                            groupList);
        break;
      }
    case ZCL_REMOVE_GROUP_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        // Command is fixed length: 3
        if (cmd->bufLen < payloadOffset + 3u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfGroupsClusterRemoveGroupResponseCallback(status,
                                                                     groupId);
        break;
      }
    default:
      {
        // Unrecognized command ID, error status will apply.
        break;
      }
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}

// Cluster: Scenes, client
EmberAfStatus emberAfScenesClusterClientCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (!cmd->mfgSpecific) {
    switch (cmd->commandId) {
    case ZCL_ADD_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneId;  // Ver.: always
        // Command is fixed length: 4
        if (cmd->bufLen < payloadOffset + 4u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        sceneId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfScenesClusterAddSceneResponseCallback(status,
                                                                  groupId,
                                                                  sceneId);
        break;
      }
    case ZCL_VIEW_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneId;  // Ver.: always
        uint16_t transitionTime;  // Ver.: always
        uint8_t* sceneName;  // Ver.: always
        uint8_t* extensionFieldSets;  // Ver.: always
        // Command is not a fixed length
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        sceneId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if ( !( status==0)) {
          // Argument is not always present:
          // - it is conditionally present based on expression: status==0
          transitionTime = 0xFFFF;
        } else {
          transitionTime = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
          payloadOffset += 2u;
        }
        if ( !( status==0)) {
          // Argument is not always present:
          // - it is conditionally present based on expression: status==0
          sceneName = NULL;
        } else {
          sceneName = emberAfGetString(cmd->buffer, payloadOffset, cmd->bufLen);
          payloadOffset += emberAfStringLength(cmd->buffer + payloadOffset) + 1u;
        }
        if ( status==0 ) {
          // Array is conditionally present based on expression: status==0
          extensionFieldSets = cmd->buffer + payloadOffset;
        } else {
          extensionFieldSets = NULL;
        }
        wasHandled = emberAfScenesClusterViewSceneResponseCallback(status,
                                                                   groupId,
                                                                   sceneId,
                                                                   transitionTime,
                                                                   sceneName,
                                                                   extensionFieldSets);
        break;
      }
    case ZCL_REMOVE_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneId;  // Ver.: always
        // Command is fixed length: 4
        if (cmd->bufLen < payloadOffset + 4u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        sceneId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfScenesClusterRemoveSceneResponseCallback(status,
                                                                     groupId,
                                                                     sceneId);
        break;
      }
    case ZCL_REMOVE_ALL_SCENES_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        // Command is fixed length: 3
        if (cmd->bufLen < payloadOffset + 3u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfScenesClusterRemoveAllScenesResponseCallback(status,
                                                                         groupId);
        break;
      }
    case ZCL_STORE_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneId;  // Ver.: always
        // Command is fixed length: 4
        if (cmd->bufLen < payloadOffset + 4u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        sceneId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfScenesClusterStoreSceneResponseCallback(status,
                                                                    groupId,
                                                                    sceneId);
        break;
      }
    case ZCL_GET_SCENE_MEMBERSHIP_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint8_t capacity;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneCount;  // Ver.: always
        uint8_t* sceneList;  // Ver.: always
        // Command is not a fixed length
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        capacity = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        if ( !( status==0)) {
          // Argument is not always present:
          // - it is conditionally present based on expression: status==0
          sceneCount = 0xFF;
        } else {
          sceneCount = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
          payloadOffset += 1u;
        }
        if ( status==0 ) {
          // Array is conditionally present based on expression: status==0
          sceneList = cmd->buffer + payloadOffset;
        } else {
          sceneList = NULL;
        }
        wasHandled = emberAfScenesClusterGetSceneMembershipResponseCallback(status,
                                                                            capacity,
                                                                            groupId,
                                                                            sceneCount,
                                                                            sceneList);
        break;
      }
    case ZCL_ENHANCED_ADD_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneId;  // Ver.: always
        // Command is fixed length: 4
        if (cmd->bufLen < payloadOffset + 4u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        sceneId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfScenesClusterEnhancedAddSceneResponseCallback(status,
                                                                          groupId,
                                                                          sceneId);
        break;
      }
    case ZCL_ENHANCED_VIEW_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupId;  // Ver.: always
        uint8_t sceneId;  // Ver.: always
        uint16_t transitionTime;  // Ver.: always
        uint8_t* sceneName;  // Ver.: always
        uint8_t* extensionFieldSets;  // Ver.: always
        // Command is not a fixed length
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        groupId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        if (cmd->bufLen < payloadOffset + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        sceneId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        transitionTime = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        if (cmd->bufLen < payloadOffset + emberAfStringLength(cmd->buffer + payloadOffset) + 1u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        sceneName = emberAfGetString(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += emberAfStringLength(cmd->buffer + payloadOffset) + 1u;
        extensionFieldSets = cmd->buffer + payloadOffset;
        wasHandled = emberAfScenesClusterEnhancedViewSceneResponseCallback(status,
                                                                           groupId,
                                                                           sceneId,
                                                                           transitionTime,
                                                                           sceneName,
                                                                           extensionFieldSets);
        break;
      }
    case ZCL_COPY_SCENE_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t status;  // Ver.: always
        uint16_t groupIdFrom;  // Ver.: always
        uint8_t sceneIdFrom;  // Ver.: always
        // Command is fixed length: 4
        if (cmd->bufLen < payloadOffset + 4u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        status = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupIdFrom = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        sceneIdFrom = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfScenesClusterCopySceneResponseCallback(status,
                                                                   groupIdFrom,
                                                                   sceneIdFrom);
        break;
      }
    default:
      {
        // Unrecognized command ID, error status will apply.
        break;
      }
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}

// Cluster: ZLL Commissioning, client
EmberAfStatus emberAfZllCommissioningClusterClientCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (!cmd->mfgSpecific) {
    switch (cmd->commandId) {
    case ZCL_ENDPOINT_INFORMATION_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t* ieeeAddress;  // Ver.: always
        uint16_t networkAddress;  // Ver.: always
        uint8_t endpointId;  // Ver.: always
        uint16_t profileId;  // Ver.: always
        uint16_t deviceId;  // Ver.: always
        uint8_t version;  // Ver.: always
        // Command is fixed length: 16
        if (cmd->bufLen < payloadOffset + 16u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        ieeeAddress = cmd->buffer + payloadOffset;
        payloadOffset += 8u;
        networkAddress = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        endpointId = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        profileId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        deviceId = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 2u;
        version = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        wasHandled = emberAfZllCommissioningClusterEndpointInformationCallback(ieeeAddress,
                                                                               networkAddress,
                                                                               endpointId,
                                                                               profileId,
                                                                               deviceId,
                                                                               version);
        break;
      }
    case ZCL_GET_GROUP_IDENTIFIERS_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t total;  // Ver.: always
        uint8_t startIndex;  // Ver.: always
        uint8_t count;  // Ver.: always
        uint8_t* groupInformationRecordList;  // Ver.: always
        // Command is fixed length: 3
        if (cmd->bufLen < payloadOffset + 3u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        total = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        startIndex = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        count = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        groupInformationRecordList = cmd->buffer + payloadOffset;
        wasHandled = emberAfZllCommissioningClusterGetGroupIdentifiersResponseCallback(total,
                                                                                       startIndex,
                                                                                       count,
                                                                                       groupInformationRecordList);
        break;
      }
    case ZCL_GET_ENDPOINT_LIST_RESPONSE_COMMAND_ID:
      {
        uint16_t payloadOffset = cmd->payloadStartIndex;
        uint8_t total;  // Ver.: always
        uint8_t startIndex;  // Ver.: always
        uint8_t count;  // Ver.: always
        uint8_t* endpointInformationRecordList;  // Ver.: always
        // Command is fixed length: 3
        if (cmd->bufLen < payloadOffset + 3u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
        total = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        startIndex = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        count = emberAfGetInt8u(cmd->buffer, payloadOffset, cmd->bufLen);
        payloadOffset += 1u;
        endpointInformationRecordList = cmd->buffer + payloadOffset;
        wasHandled = emberAfZllCommissioningClusterGetEndpointListResponseCallback(total,
                                                                                   startIndex,
                                                                                   count,
                                                                                   endpointInformationRecordList);
        break;
      }
    default:
      {
        // Unrecognized command ID, error status will apply.
        break;
      }
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}

// Cluster: Manager, server
EmberAfStatus emberAfManagerServerCommandParse(EmberAfClusterCommand *cmd)
{
  bool wasHandled = false;
  if (cmd->mfgSpecific) {
    if (cmd->mfgCode == 0x10A2 && cmd->commandId == ZCL_GET_REPORT_TIME_COMMAND_ID) {
      uint16_t payloadOffset = cmd->payloadStartIndex;
      uint16_t reportStr;  // Ver.: always
      // Command is fixed length: 2
      if (cmd->bufLen < payloadOffset + 2u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
      reportStr = emberAfGetInt16u(cmd->buffer, payloadOffset, cmd->bufLen);
      wasHandled = emberAfManagerGetReportTimeCallback(reportStr);
    } else if (cmd->mfgCode == 0x10A2 && cmd->commandId == ZCL_GET_PING_COMMAND_ID) {
      uint16_t payloadOffset = cmd->payloadStartIndex;
      uint8_t* Ping;  // Ver.: always
      // Command is fixed length: 8
      if (cmd->bufLen < payloadOffset + 8u) { return EMBER_ZCL_STATUS_MALFORMED_COMMAND; }
      Ping = cmd->buffer + payloadOffset;
      wasHandled = emberAfManagerGetPingCallback(Ping);
    }
  }
  return status(wasHandled, true, cmd->mfgSpecific);
}
