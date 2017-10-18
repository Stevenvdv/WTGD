#ifndef _TURN_H_
#define _TURN_H_

class Turn : public ICommand
{
	public:
		//Method where the Turn Command will be execute.
		void Execute();
		
		//Constructor
		Turn(double Degrees, WillyController* Controller);

		//Default ToString Methods
		std::string toString();
		ostream& operator<< (Turn& obj) {
			return cout << obj.toString();
		}
		
	private:
		double _degrees;
		WillyController* _controller;
};

#endif