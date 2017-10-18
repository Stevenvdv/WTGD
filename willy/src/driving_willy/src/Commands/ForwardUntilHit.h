#ifndef _FORWARD_UNTIL_HIT_
#define _FORWARD_UNTIL_HIT_

class ForwardUntilHit : public ICommand
{
	public:
		//Method where the ForwardUntilHit Command will be execute.
		void Execute();

		//Constructor
		ForwardUntilHit(WillyController* Controller);

		//Default ToString Methods
		std::string toString();
		ostream& operator<< (Forward& obj) {
			return cout << obj.toString();
		}

	private:
		WillyController* _controller;
};

#endif