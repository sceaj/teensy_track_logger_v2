/*
 * generic_hashmap.h
 *
 *  Created on: Oct 8, 2020
 *      Author: jrosen
 */

#ifndef MAPS_GENERIC_HASHMAP_H_
#define MAPS_GENERIC_HASHMAP_H_

#include "fsl_common.h"

/*!*********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public macro definitions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
/*! @brief The map status */
typedef enum _map_status
{
    kMAP_Ok             = kStatus_Success,                   /*!< Success */
    kMAP_DuplicateError = MAKE_STATUS(kStatusGroup_LIST, 1), /*!< Duplicate Error */
    kMAP_Full           = MAKE_STATUS(kStatusGroup_LIST, 2), /*!< FULL */
    kMAP_Empty          = MAKE_STATUS(kStatusGroup_LIST, 3), /*!< Empty */
    kMAP_OrphanElement  = MAKE_STATUS(kStatusGroup_LIST, 4), /*!< Orphan Element */
    kMAP_NotSupport     = MAKE_STATUS(kStatusGroup_LIST, 5), /*!< Not Support  */
} map_status_t;

/*! @brief The map structure*/
typedef struct map_label
{
    struct list_element_tag *head; /*!< list head */
    struct list_element_tag *tail; /*!< list tail */
    uint16_t size;                 /*!< map size */
    uint16_t max;                  /*!< map max number of elements */
} map_label_t, *list_map_t;

/*! @brief The map element*/
typedef struct list_element_tag
{
    struct list_element_tag *next; /*!< next list element   */
    struct list_label *list;       /*!< pointer to the list */
} list_element_t, *list_element_handle_t;
/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */
/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* _cplusplus */
/*!
 * @brief Initialize the map.
 *
 * This function initializes a map.
 *
 * @param map - Map handle to initialize.
 */
void MAP_Init(map_handle_t map, );

/*!
 * @brief Links element to the head of the list.
 *
 * @param map - Handle of the map.
 * @param key - Pointer to a string that is the key for data lookup.
 * @param data - Pointer to the data to be stored.
 * @retval kLIST_Full if list is full, kLIST_Ok if insertion was successful.
 */
list_status_t MAP_Put(map_handle_t map, char* key, void* data);

/*!
 * @brief Looks up  element to the tail of the list.
 *
 * @param list - Handle of the list.
 * @param element - Handle of the element.
 * @retval kLIST_Full if list is full, kLIST_Ok if insertion was successful.
 */
list_status_t MAP_Get(map_handle_t map, char* key, void** dataref);

/*!
 * @brief Unlinks element from the head of the list.
 *
 * @param list - Handle of the list.
 *
 * @retval NULL if list is empty, handle of removed element(pointer) if removal was successful.
 */
list_element_handle_t MAP_Remove(list_handle_t list);

/*!
 * @brief Gets next element handle for given element handle.
 *
 * @param element - Handle of the element.
 *
 * @retval NULL if list is empty, handle of removed element(pointer) if removal was successful.
 */
list_element_handle_t LIST_GetNext(list_element_handle_t element);

/*!
 * @brief Gets previous element handle for given element handle.
 *
 * @param element - Handle of the element.
 *
 * @retval NULL if list is empty, handle of removed element(pointer) if removal was successful.
 */
list_element_handle_t LIST_GetPrev(list_element_handle_t element);

/*!
 * @brief Unlinks an element from its list.
 *
 * @param element - Handle of the element.
 *
 * @retval kLIST_OrphanElement if element is not part of any list.
 * @retval kLIST_Ok if removal was successful.
 */
list_status_t LIST_RemoveElement(list_element_handle_t element);

/*!
 * @brief Links an element in the previous position relative to a given member of a list.
 *
 * @param list - Handle of the list.
 * @param element - Handle of the element.
 * @param newElement - New element to insert before the given member.
 *
 * @retval kLIST_OrphanElement if element is not part of any list.
 * @retval kLIST_Ok if removal was successful.
 */
list_status_t LIST_AddPrevElement(list_element_handle_t element, list_element_handle_t newElement);

/*!
 * @brief Gets the current size of a list.
 *
 * @param list - Handle of the list.
 *
 * @retval Current size of the list.
 */
uint32_t LIST_GetSize(list_handle_t list);

/*!
 * @brief Gets the number of free places in the list.
 *
 * @param list - Handle of the list.
 *
 * @retval Available size of the list.
 */
uint32_t LIST_GetAvailableSize(list_handle_t list);

/* @} */

#if defined(__cplusplus)
}
#endif

#endif /* MAPS_GENERIC_HASHMAP_H_ */
