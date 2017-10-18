#ifndef _BACKWARD_H_
#define _BACKWARD_H_

class Backward : public ICommand
{
	public:
		//Declataratio of Function for executing Bacward Command
		void Execute();
		
		//Constructor
		Backward(double Meters, WillyController* Controller);

		//Declataratio of toString Function
		std::string toString();
		ostream& operator<< (Forward& obj) {
			return cout << obj.toString();
		}

	private:
		double _meters;
		WillyController* _controller;
};

#endif