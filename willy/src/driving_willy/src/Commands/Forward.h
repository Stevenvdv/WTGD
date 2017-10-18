#ifndef _FORWARD_H_
#define _FORWARD_H_

class Forward : public ICommand
{
	public:
		//Method where the Forward Command will be execute.
		void Execute();

		//Constructor
		Forward(double Meters, WillyController* Controller);

		//Default ToString Methods
		std::string toString();
		ostream& operator<< (Forward& obj) {
			return cout << obj.toString();
		}
	private:
		double _meters;
		WillyController* _controller;
};

#endif