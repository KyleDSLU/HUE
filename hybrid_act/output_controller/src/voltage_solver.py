
import numpy as np
import scipy.optimize

class Voltage_Solver():

    def __init__(self,first_gain,second_gain):
        self.x = np.arange(100)
        self.AD9833sin_in = 0.3*np.sin(self.x/float(len(self.x)) * 2 * np.pi) + 0.315
        self.first_gain = first_gain
        self.second_gain = second_gain

    def solve(self,voltage_avg_array,voltage_amp_array):
        # save A0 and A1 for each case
        self.solutions = np.zeros([voltage_avg_array.shape[0], 2])
        for i in range(len(voltage_avg_array)):
            avg = voltage_avg_array[i]
            amp = voltage_amp_array[i]
            a0,a1 = self.solve_sinusoid(avg,amp)
            self.solutions[i][:] = [a0,a1]
        return self.solutions

    def solve_sinusoid(self,avg,amp):
        A = [1,1]
        opt = scipy.optimize.fmin(self.cost, A, args=(avg,amp),disp=False)
        return opt

    def cost(self,A,avg,amp):
        sin = (self.AD9833sin_in*self.first_gain*A[0]/10. + A[1]) * self.second_gain
        sin_flip = (10-sin)

        resultant = sin-sin_flip
        cost1 = (np.average(resultant)-avg)**2
        cost2 = ((max(resultant)-np.average(resultant))-amp)**2
        cost = cost1+cost2

        return cost

def interpolate(avg_desired, amp_desired):
    solutions_read = np.genfromtxt('solutions.csv', delimiter=',')
    key_read = np.genfromtxt('key.csv', delimiter=',')
    avg_read = np.genfromtxt('avg.csv', delimiter=',')
    amp_read = np.genfromtxt('amp.csv', delimiter=',')

    solutions_read.shape = tuple(key_read.astype(int))

    avg_ind = np.searchsorted(avg_read,avg_desired,'right')
    amp_ind = np.searchsorted(amp_read,amp_desired,'right')

    if amp_read[amp_ind-1] == amp_desired:
        solution = np.zeros_like(solutions_read[amp_ind-1])
        solution[:] = solutions[amp_ind-1]
        print("HERE")

    else:
        solution = np.zeros_like(solutions_read[amp_ind])
        amp_spacing = amp_read[amp_ind] - amp_read[amp_ind-1]
        amp_diff = amp_desired - amp_read[amp_ind-1]
        for i in range(len(solution)):
            ends = [solutions[:,i][amp_ind-1],solutions[:,i][amp_ind]]
            print(amp_diff, amp_spacing)
            solution[i] = np.interp(amp_diff, np.linspace(0,amp_spacing,len(ends)), ends)
        
    return solution


def gain_cost(gain, avg, amp):
    solver = Voltage_Solver(gain)
    solutions = solver.solve(avg,amp)
    ind = [np.where((solutions<0)),np.where((solutions>5))]
    cost = 0
    for i in range(len(ind)):
        cost += ind[0][i].shape[0] + ind[1][i].shape[0]
    return cost
    

if __name__ == "__main__":
    # solver = Voltage_Solver(-4.7)
    avg = np.arange(0.25,6.75,0.25)
    amp = 0.6*avg

    solver = Voltage_Solver(-4.7,9.1)
    solutions = solver.solve(avg,amp)
    key = np.array(solutions.shape)

    np.savetxt("key.csv", key, delimiter=',')
    np.savetxt("amp.csv", amp, delimiter=',')
    np.savetxt("avg.csv", avg, delimiter=',')
    np.savetxt("solutions.csv", solutions.flatten(), delimiter=',')

    avg_desired = 5.2783
    amp_desired = avg_desired*0.6

    print(interpolate(avg_desired, amp_desired))


