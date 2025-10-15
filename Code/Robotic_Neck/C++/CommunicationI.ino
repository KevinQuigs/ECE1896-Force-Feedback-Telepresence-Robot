//Interface for communication

class CommunicationI {

  public:
    virtual bool isOpen() = 0;

    virtual void start() = 0;

    virtual void output(char* data) = 0;

    virtual bool readData(char* input) = 0;
};