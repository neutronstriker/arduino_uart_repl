/*
 * myStack.cpp
 *
 * Created: 30-05-2015 16:11:17
 *  Author: neutron
 */ 
#include "myStack.h"

myStack::myStack(uint8_t size)
{
	array_size = size;
	char stack_data[size];//check if this is right or need to use "new" here
	array = stack_data;
	top_pointer = -1;
}

myStack::~myStack()
{
	//Destructor
	//Nothing to do.
}


bool myStack::isFull()
{
		if(top_pointer == array_size-1)
		return true;
		else return false;
		
}

bool myStack::isEmpty()
{
	if(top_pointer==-1)
	return true;
	else return false;
}
	
	int myStack::push(unsigned char element)
	{
		if(top_pointer == (array_size-1))
		{
			//stack is full
			return 0;
		}
		top_pointer = top_pointer+1;
		array[top_pointer] = element;
		return 1;
	}

	unsigned char myStack::pop()
	{
		if(top_pointer == -1)
		{
			//stack is empty
			return 0;
		}
		return(array[top_pointer--]);

	}
	