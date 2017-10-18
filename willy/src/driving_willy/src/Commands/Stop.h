#ifndef _STOP_H_
#define _STOP_H_

class Stop : public ICommand
{
	public:
		void Execute();

		Stop(int Time);	

		std::string toString();

		ostream& operator<< (Stop& obj) {
			return cout << obj.toString();
		}
	private: 
		int _time;
};

#endif