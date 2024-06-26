/******************************************************************************
* Includes
*******************************************************************************/
#include "fifo.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
int fifo_init(fifo_t* p_fifo)
{
    assert(p_fifo != NULL);
    p_fifo->ll_obj_p = ll_create();
    return 0;
}

void fifo_put(fifo_t* p_fifo,void* p_data)
{
    assert(p_fifo != NULL);
    assert(p_data != NULL);
    ll_push_back(p_fifo->ll_obj_p,p_data);
}

void* fifo_peek(fifo_t* p_fifo)
{
    assert(p_fifo != NULL);
    return ll_get_front(p_fifo->ll_obj_p);
}

void* fifo_get(fifo_t* p_fifo)
{
    assert(p_fifo != NULL);
    void* p_data = ll_get_front(p_fifo->ll_obj_p);
    ll_pop_front(p_fifo->ll_obj_p);
    return p_data;
}

bool fifo_is_empty(fifo_t* p_fifo)
{
    assert(p_fifo != NULL);
    return ll_is_empty(p_fifo->ll_obj_p);
}


