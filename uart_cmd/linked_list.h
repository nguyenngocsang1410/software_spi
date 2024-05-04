#ifndef LINKED_LIST_H_
#define LINKED_LIST_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct my_list
{
    void* key_p;
    struct my_list *next_p;
} my_ll_t;

typedef struct ll_object
{
	/* Attribute */
	my_ll_t* head_p;
	my_ll_t* tail_p;
} ll_object_t;

/******************************************************************************
* Variables
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
ll_object_t* ll_create(void);
void* ll_get_front(ll_object_t* obj_p);
void* ll_get_back(ll_object_t* obj_p);
void  ll_pop_front(ll_object_t* obj_p);
void  ll_pop_back(ll_object_t* obj_p);
void  ll_print_list(ll_object_t* obj_p);
void  ll_push_front(ll_object_t* obj_p, void* key_p);
void  ll_push_back(ll_object_t* obj_p, void* key_p);
bool ll_is_exist(ll_object_t* obj_p, void* key_p);
bool ll_is_empty(ll_object_t* obj_p);


#endif /* LINKED_LIST_H_ */
