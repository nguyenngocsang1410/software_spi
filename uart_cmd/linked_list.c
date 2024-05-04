/******************************************************************************
* Includes
*******************************************************************************/
#include "linked_list.h"

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
void* ll_get_front(ll_object_t* obj_p)
{
	return obj_p->head_p->key_p;
}

void* ll_get_back(ll_object_t* obj_p)
{
	return obj_p->tail_p->key_p;
}

void ll_push_front(ll_object_t* obj_p, void* key_p)
{
	my_ll_t* p_new_node = (my_ll_t*) malloc( sizeof(my_ll_t) );
	assert(p_new_node != NULL);
	p_new_node->key_p = key_p;
	p_new_node->next_p = obj_p->head_p;
	obj_p->head_p = p_new_node;
    if(obj_p->tail_p == NULL) obj_p->tail_p = obj_p->head_p;
}

void ll_push_back(ll_object_t* obj_p, void* key_p)
{
	my_ll_t* p_new_node = (my_ll_t*) malloc( sizeof(my_ll_t) );
	assert(p_new_node != NULL);
	p_new_node->key_p = key_p;
	p_new_node->next_p = NULL;
	if(obj_p->head_p == NULL)
	{
		obj_p->head_p = p_new_node;
		obj_p->tail_p = obj_p->head_p;
	}
	else
	{
		obj_p->tail_p->next_p = p_new_node;
		obj_p->tail_p = p_new_node;
	}
}

void ll_pop_front(ll_object_t* obj_p)
{
	assert(obj_p->head_p != NULL);
	my_ll_t *p_curhead = obj_p->head_p;
	obj_p->head_p = obj_p->head_p->next_p;
	if(obj_p->head_p == NULL) obj_p->tail_p = NULL;
	free(p_curhead);
}

void ll_pop_back(ll_object_t* obj_p)
{
	if(obj_p->head_p == NULL)
		printf("There is nothing to pop \n");
	else
	{
		my_ll_t* p_temp=obj_p->head_p;
		if (obj_p->head_p == obj_p->tail_p)
		{
			obj_p->head_p = NULL;
			obj_p->tail_p = NULL;
		}
		else
		{
			while(p_temp->next_p != obj_p->tail_p)
			{
				p_temp = p_temp->next_p;
			}
			p_temp->next_p = NULL;	/* Update node before tail */
			obj_p->tail_p = p_temp;
			p_temp = p_temp->next_p;
		}
		free(p_temp);
	}
}

void ll_print_list(ll_object_t* obj_p)
{
	printf("\n");
	printf("HeadPt: %p \n", obj_p->head_p);
	my_ll_t* p_cur = obj_p->head_p;
	printf("------------------------------------------------------- \n");
	while(1)
	{
		if(p_cur != NULL)
		{
			printf("Address %p --- Value: %3d --- Next: %p \n", p_cur, *((uint8_t*)p_cur->key_p), p_cur->next_p);
			p_cur = p_cur->next_p;
		}
		else
		{
			break;
		}
	}
	printf("------------------------------------------------------- \n");
	printf("TailPt: %p \n",obj_p->tail_p);
	printf("\n");
}

bool ll_is_empty(ll_object_t* obj_p)
{
	if(obj_p->head_p == NULL)
		return true;
	else
		return false;
}

bool ll_is_exist(ll_object_t* obj_p, void* key_p)
{
	my_ll_t* p_cur = obj_p->head_p;
	while(p_cur != NULL)
	{
		if (p_cur->key_p != key_p)
			p_cur = p_cur->next_p;
		else
			return true;
	}
	return false;
}

ll_object_t* ll_create(void)
{
	ll_object_t *p_ll = malloc(sizeof (ll_object_t));
	assert(p_ll != NULL);
	p_ll->head_p= NULL;
	p_ll->tail_p= NULL;
	return p_ll;
}
