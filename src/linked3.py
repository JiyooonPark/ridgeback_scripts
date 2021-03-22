class Node:
    def __init__(self, d=None, n=None):
        self.data = d
        self.next = n

    def __str__(self):
        return "(" + str(self.data) + ")"


class CircularlinkedList:
    def __init__(self, r=None):
        self.head = r
        self.size = 0

    def add(self, d):
        if self.size == 0:
            self.head = Node(d)
            self.head.next = self.head
        else:
            temp = self.head.next
            new_node = Node(d, self.head.next)
            while temp.next != self.head:
                temp = temp.next
            temp.next = new_node
            temp.next.next = self.head
        self.size += 1
        # print("head :", str(self.head.data))

    def remove(self, d):
        node = self.head
        prev = None
        while True:
            if node.data == d:
                if prev is not None:
                    prev.next = node.next
                else:
                    while node.next != self.head:
                        node = node.next
                    node.next = self.head.next
                    self.head = self.head.next
                self.size -= 1
                return True
            elif node.next == self.head:
                return False
            prev = node
            node = node.next

    def print_list(self):
        if self.head is None:
            return
        node = self.head
        print(node, end="->")
        while node.next != self.head:
            node = node.next
            if node.next == self.head:
                print(node)
            else:
                print(node, end="->")


def candle(n, k):
    L = buildList(n)
    # print(" list made :", end="")
    # L.print_list()
    return runSimulation(L, n, k)


def buildList(n):
    list = CircularlinkedList()
    for i in range(1, n + 1):
        list.add(i)
    return list


def runSimulation(L, n, k):
    p = L.head

    while p != p.next:
        L.print_list()
        for i in range(k - 1):
            # print("i: ", str(i))
            p = p.next
        # print("p is:", str(p.data))
        # print("removing: ", str(p.next.data))
        L.remove(p.next.data)
        # print("p is now:", str(p.data))
        p = p.next
        # print("p is when entering the loop:", str(p.data))
    return p.data


if __name__ == "__main__":
    print(candle(7, 3), "번 촛불")
