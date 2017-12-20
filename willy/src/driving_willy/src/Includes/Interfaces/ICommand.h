#ifndef _ICOMMAND_H_
#define _ICOMMAND_H_

//This class in an interface where Every Command should implement
class ICommand
{
	public:
		virtual void Execute() = 0;
};

#endif