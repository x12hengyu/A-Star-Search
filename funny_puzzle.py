import heapq
import numpy as np

def find_pos(num, state):
    for i in range(len(state)):
        for j in range(len(state[0])):
            if state[i][j] == num:
                return i, j

def manhattan(state):
    goal = [[1,2,3],[4,5,6],[7,8,0]]
    state = np.reshape(state,(3,3))
    dist = []
    for i in range(0, len(state)):
        for j in range(0, len(state[i])):
            if state[i][j] == goal[i][j] or state[i][j] == 0:
                continue
            else:
                i_goal, j_goal = find_pos(state[i][j], goal)
                
                distance = abs(i - i_goal) + abs(j - j_goal)
                dist.append(distance)
    return sum(dist)
    
def print_succ(state):
    r = get_succ(state)
    for i in r:
        print(i, "h={}".format(manhattan(i)))

def get_succ(state):
    state = np.reshape(state,(3,3))
    l = len(state)
    i, j = find_pos(0, state)

    r = []
    if i - 1 >= 0:
        state_temp = state.copy()
        state_temp[i][j] = state_temp[i-1][j]
        state_temp[i-1][j] = 0
        r.append(np.ndarray.tolist(np.reshape(state_temp, (1,9)))[0])
    if i + 1 < l:
        state_temp = state.copy()
        state_temp[i][j] = state_temp[i+1][j]
        state_temp[i+1][j] = 0
        r.append(np.ndarray.tolist(np.reshape(state_temp, (1,9)))[0])
    if j - 1 >= 0:
        state_temp = state.copy()
        state_temp[i][j] = state_temp[i][j-1]
        state_temp[i][j-1] = 0
        r.append(np.ndarray.tolist(np.reshape(state_temp, (1,9)))[0])
    if j + 1 < l:
        state_temp = state.copy()
        state_temp[i][j] = state_temp[i][j+1]
        state_temp[i][j+1] = 0
        r.append(np.ndarray.tolist(np.reshape(state_temp, (1,9)))[0])

    return sorted(r)

def get_node(nodes, state):
    for n in nodes:
        if state == n[1]:
            return n
    return 0

def traceback(close_nodes):
    path = []
    last = len(close_nodes) - 1
    parent = close_nodes[last][2][2]
    path.append(close_nodes[last])
    while parent != -1:
        last = parent
        parent = close_nodes[last][2][2]
        path.append(close_nodes[last])
    return path

def solve(state):
    open_nodes, close_nodes = [], []
    path = []
    # push initial state to priority queue
    heapq.heappush(open_nodes, (0+manhattan(state), state, (0, manhattan(state), -1)))
    # loop runs when open_nodes is not empty
    while len(open_nodes) != 0:
        cur = heapq.heappop(open_nodes)
        cur_state = cur[1]
        close_nodes.append(cur)
        path.append(cur_state)
        # when manhattan distance is 0 we reach the goal
        if cur_state == [1, 2, 3, 4, 5, 6, 7, 8, 0]:  # terminate when answer is find
            path = traceback(close_nodes)
            break
        # else do A* search
        for n in get_succ(cur_state):
            if n not in path:
                g_move = cur[2][0] + 1
                h = manhattan(n)
                heapq.heappush(open_nodes, (g_move + h, n, (g_move, h, len(close_nodes)-1)))

    step = 0
    for i in reversed(path):  # print the path
        print("{} h={} moves:{}".format(i[1], i[2][1], step))
        step += 1
    return 0

if __name__ == "__main__":
    solve([4,3,8,5,1,6,7,2,0])