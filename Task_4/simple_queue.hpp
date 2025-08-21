template<typename T, int MAX_SIZE>
class SimpleQueue {
public:
  SimpleQueue() : front(0), back(0), count(0) {}

  bool isEmpty() const { return count == 0; }
  bool isFull() const { return count == MAX_SIZE; }

  void enqueue(const T& item) {
    if (!isFull()) {
      data[back] = item;
      back = (back + 1) % MAX_SIZE;
      count++;
    }
  }

  void dequeue(T& item) {
    if (!isEmpty()) {
      item = data[front];
      front = (front + 1) % MAX_SIZE;
      count--;
    }
  }

private:
  T data[MAX_SIZE];
  int front, back, count;
};