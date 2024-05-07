#ifndef FIFO_H_
#define FIFO_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "linked_list.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct fifo_obj
{
    ll_object_t* ll_obj_p;
} fifo_t;
/******************************************************************************
* Variables
*******************************************************************************/

/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
int fifo_init(fifo_t* p_fifo);
bool fifo_is_empty(fifo_t* p_fifo);
void fifo_put(fifo_t* p_fifo,void* p_data);
void* fifo_get(fifo_t* p_fifo);
void* fifo_peek(fifo_t* p_fifo);


#endif /* FIFO_H_ */