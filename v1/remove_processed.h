#ifndef REMOVEDPROCESSED
#define REMOVEDPROCESSED

class remove_processed
{
public:

	remove_processed()
	{
	}

	virtual ~remove_processed()
	{
	}

	bool operator() (Position loc)
	{
		return false;
	}
};

#endif //REMOVEDPROCESSED

