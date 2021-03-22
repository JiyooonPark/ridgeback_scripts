# Represents the node of list.
class Node:
    def __init__(self, data):
        self.data = data
        self.next = None


class CreateList:
    # Declaring head and tail pointer as null.
    def __init__(self):
        self.head = Node(None)
        self.tail = Node(None)
        self.head.next = self.tail
        self.tail.next = self.head

    # This function will add the new node at the end of the list.
    def add(self, data):
        newNode = Node(data)
        # Checks if the list is empty.
        if self.head.data is None:
            # If list is empty, both head and tail would point to new node.
            self.head = newNode
            self.tail = newNode
            newNode.next = self.head
        else:
            # tail will point to new node.
            self.tail.next = newNode
            # New node will become new tail.
            self.tail = newNode
            # Since, it is circular linked list tail will point to head.
            self.tail.next = self.head

    def removeData(self, data):
        temp = self.head
        while temp.next.data != data:
            temp = temp.next
        if temp.next == self.tail:
            self.tail = temp
        if temp.next == self.head:
            self.head = self.head.next
        temp.next = temp.next.next

    # Displays all the nodes in the list
    def display(self):
        current = self.head
        if self.head is None:
            print("List is empty")
            return
        else:
            print("Nodes of the circular linked list: ")
            # Prints each node by incrementing pointer.
            print(current.data, end="->"),
            while current.next != self.head:
                current = current.next
                print(current.data, end="->"),
            print("\n")
            print("head is :", str(self.head.data), "tail is :", str(self.tail.data))


def candle(n, k):
    L = buildList(n)
    L.display()
    return runSimulation(L, n, k)


def buildList(n):
    list = CreateList()
    for i in range(1, n):
        list.add(i)
    return list


def runSimulation(L, n, k):
    p = L.head

    while p != p.next:
        L.display()
        for i in range(k - 2):
            print("i: ", str(i))

            p = p.next
            print(str(p.data), str(p.next.data))
        L.removeData(p.next.next)
    return p.data


if __name__ == "__main__":
    print(candle(7, 3))
