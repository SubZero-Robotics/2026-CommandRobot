package frc.robot.utils;

public class RingBuffer<T> {
    private int m_tail;
    private int m_head;
    private int m_length;

    private Object[] m_elements;

    public RingBuffer(int maxLength) {

        if (maxLength < 1) {
            System.out.println("Cannot create ring buffer with a max length of < 1.");
            return;
        }

        m_elements = new Object[maxLength];

        m_head = 0;
        m_tail = 0;
        m_length = maxLength;
    }

    public void push(T element) {
        if (m_head - m_tail == m_length) {
            pop();
        }

        m_elements[getTrueIndex(m_head++)] = element;
    }

    public T pop() {
        if (m_tail == m_head) {
            System.out.println("Cannot pop from empty ring buffer.");
            return null;
        }

        return (T) m_elements[getTrueIndex(m_tail++)];
    }

    public T get(int index) {
        int trueIndex = m_tail + index;
        if (trueIndex > m_head) {
            System.out.println("Index " + index + " is out of bounds of the ring buffer.");
            return null;
        }

        return (T) m_elements[getTrueIndex(trueIndex)];
    }

    public int getLength() {
        return m_head - m_tail;
    }

    @Override
    public String toString() {
        String str = "{ ";
        for (int i = m_tail; i < m_head; i++) {
            str += m_elements[getTrueIndex(i)] + " ";
        }
        return str + "}";
    }

    private int getTrueIndex(int index) {
        return index % m_elements.length;
    }
}