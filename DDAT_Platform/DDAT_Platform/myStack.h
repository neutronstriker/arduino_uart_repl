/*
 * myStack.h
 *
 * Created: 30-05-2015 16:12:14
 *  Author: neutron
 */ 


#ifndef MYSTACK_H_
#define MYSTACK_H_
#define uint8_t unsigned char //otherwise throwing error that uint8_t not recognised
//but i did not want to include stdint.h again here. However there will not any problem 
//even though we include it because it already will be within the include only once block.

class myStack{ /*i  would like to use the word stack instead of myStack but is a reserved keyword
		So using it creates problems. */	
	private:
	
		int top_pointer;
		char *array;
		uint8_t array_size;
		
	public:
	
		myStack(uint8_t size);
		
		~myStack();
		
		int push(unsigned char);
		unsigned char pop();
		
		bool isFull();
		
		bool isEmpty();
	};



#endif /* MYSTACK_H_ */