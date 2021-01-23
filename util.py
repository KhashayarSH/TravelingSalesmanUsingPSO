import math
import random
import matplotlib.pyplot as plt
import csv

# class which represents each city
class City:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # computes distance between current city and given city
    def distance(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    # when City is to be presented as a string like printing it,
    # it is printed as x, y
    def __repr__(self):
        return f"({self.x}, {self.y})"

# reads the input file with given number of cities
def read_cities(size):
    cities = []
    with open(f'input{size}.txt', 'r') as handle:
        lines = handle.readlines()
        for line in lines:
            if len(line.split()) == 2:
                x, y = map(float, line.split())
                cities.append(City(x, y))
    return cities

# computes path_cost of a permutation
def path_cost(route):
    route_cost = 0
    number_of_cities = len(route)
    for i in range(number_of_cities):
        route_cost += route[i].distance(route[(i+1)%number_of_cities])
    return route_cost


def visualize_tsp(title, cities):
    fig = plt.figure()
    fig.suptitle(title)
    x_list, y_list = [], []
    for city in cities:
        x_list.append(city.x)
        y_list.append(city.y)
    x_list.append(cities[0].x)
    y_list.append(cities[0].y)

    plt.plot(x_list, y_list, 'ro')
    plt.plot(x_list, y_list, 'g')
    plt.show(block=True)

# prints best,average and worst path_cost of each iteration
def print_iteration_data(pso,size):
    test = []
    for i in range(len(pso.best_of_generation)):
        test.append([pso.best_of_generation[i],pso.average_of_generation[i],pso.worst_of_generation[i]])
    with open(f'PSO{size}.txt', 'w', newline='') as csvfile:
        spamwriter = csv.writer(csvfile, delimiter=',',quotechar='\'')
        spamwriter.writerows(test)
