# Structure for a Node
class Node:
    # Constructor to create  a new node
    def __init__(self, data):
        self.data = data 
        self.next = None
 
class CircularLinkedList:
    # Constructor to create a empty circular linked list
    def __init__(self):
        self.head = None
    # Function to insert a node at the beginning of a
    # circular linked list
    def push(self, data):
        ptr1 = Node(data)
        temp = self.head
         
        ptr1.next = self.head
 
        # If linked list is not None then set the next of
        # last node
        if self.head is not None:
            while(temp.next != self.head):
                temp = temp.next
            temp.next = ptr1
 
        else:
            ptr1.next = ptr1 # For the first node
 
        self.head = ptr1 

    def pushEnd(self, data):
        ptr1 = Node(data)
        ptr2 = self.head
        if ptr2 != None:
            ptr2= ptr2.next

        while(ptr2!=self.head):
            ptr2 = ptr2.next
        ptr2.next = ptr1 
        ptr1.next = self.head
 
        # If linked list is not None then set the next of
        # last node
        # if self.head is not None:
        #     while(temp.next != self.head):
        #         temp = temp.next
        #     temp.next = ptr1
 
        # else:
        #     ptr1.next = ptr1 # For the first node
 

 
    # Function to print nodes in a given circular linked list
    def printList(self):
        temp = self.head
        if self.head is not None:
            while(True):
                print ("%d" %(temp.data), end="->")
                temp = temp.next
                if (temp == self.head):
                    print("\n")
                    break
 
 
def candle(n, k):
    L = buildList(n)
    L.printList()
    # return runSimulation(L, n, k)
def buildList( n):
    list = CircularLinkedList()
    for i in range(1, n):
        list.pushEnd(i)
    list.printList()
    return list
def runSimulation(L, n, k):
    p = L.head
    while( p != p.next):
        L.printList()
        for i in range( 1, k-1):
            p = p.next
        pnext = p.next
        p.next = p.next.next
        p = p.next
    return p.data

if __name__=="__main__":
    print(candle(7,3))

# cllist = CircularLinkedList()
 
# # Created linked list will be 11->2->56->12
# cllist.push(12)
# cllist.push(56)
# cllist.push(2)
# cllist.push(11)
 
# print ("Contents of circular Linked List")
# cllist.printList()
           
# # This code is contributed by 
# # Nikhil Kumar Singh(nickzuck_007)