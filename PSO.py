import random
import math
import matplotlib.pyplot as plt
from util import City, read_cities, path_cost, print_iteration_data

# class Particle for PSO
class Particle:
    # each particle has a  route, personal best (route), current cost, personal best cost, velocity
    def __init__(self, route, cost=None):
        self.route = route
        self.pbest = route
        self.current_cost = cost if cost else self.path_cost()
        self.pbest_cost = cost if cost else self.path_cost()
        self.velocity = []

    def clear_velocity(self):
        self.velocity.clear()

    def update_costs_and_pbest(self):
        self.current_cost = self.path_cost()
        if self.current_cost < self.pbest_cost:
            self.pbest = self.route
            self.pbest_cost = self.current_cost
    # cost is calculated using path cost from util
    def path_cost(self):
        return path_cost(self.route)

# Class PSO algorithm for TSP
class PSO:
    def __init__(self, iterations, population_size, gbest_probability=1.0, pbest_probability=1.0, cities=None):
        # list of cities (initial permutation of cities)
        self.cities = cities
        self.gbest = None
        self.gbest_of_generation = []
        self.best_of_generation = []
        self.worst_of_generation = []
        self.average_of_generation = []
        self.iterations = iterations
        self.population_size = population_size
        self.particles = []
        self.gbest_probability = gbest_probability
        self.pbest_probability = pbest_probability
        # initiate population
        solutions = self.initial_population()
        self.particles = [Particle(route=solution) for solution in solutions]

    # returns random permutation of cities
    def random_route(self):
        return random.sample(self.cities, len(self.cities))

    # returns specified(population_size) number of random permutations of cities
    # also adds the greedy solution to the population
    def initial_population(self):
        random_population = [self.random_route() for _ in range(self.population_size - 1)]
        greedy = self.greedy_route()
        random_population.append(greedy)
        return random_population

    # computes a greedy route by finding closest city at each step
    def greedy_route(self):
        unvisited = self.cities[:]
        del unvisited[0]
        route = [self.cities[0]]
        while len(unvisited):
            index, nearest_city = min(enumerate(unvisited), key=lambda item: item[1].distance(route[-1]))
            route.append(nearest_city)
            del unvisited[index]
        return route

    # executes PSO
    def run(self):
        # PSO is executed for specified(iterations) number of iterations
        for t in range(self.iterations):
            # store gbest, best, worst and average fitness of current iteration
            self.gbest = min(self.particles, key=lambda p: p.pbest_cost)
            self.gbest_of_generation.append(self.gbest.pbest_cost)
            best = min(self.particles, key=lambda p: p.current_cost)
            self.best_of_generation.append(best.current_cost)
            worst = max(self.particles, key=lambda p: p.current_cost)
            self.worst_of_generation.append(worst.current_cost)
            average = 0
            for particle in self.particles:
                average += particle.current_cost
            average /= len(self.particles)
            self.average_of_generation.append(average)
            # compute velocity for each particle and apply it on them
            for particle in self.particles:
                particle.clear_velocity()
                temp_velocity = []
                gbest = self.gbest.pbest[:]
                new_route = particle.route[:]

                # if place of a city is not same as pbest the swap that fixes
                # the position is added to velocity
                for i in range(len(self.cities)):
                    if new_route[i] != particle.pbest[i]:
                        swap = (i, particle.pbest.index(new_route[i]), self.pbest_probability)
                        temp_velocity.append(swap)

                # if place of a city is not same as gbest the swap that fixes
                # the position is added to velocity
                for i in range(len(self.cities)):
                    if new_route[i] != gbest[i]:
                        swap = (i, gbest.index(new_route[i]), self.gbest_probability)
                        temp_velocity.append(swap)

                particle.velocity = temp_velocity

                # each swap in velocity is applied with probability of pbest or gbest accordingly
                for swap in temp_velocity:
                    if random.random() <= swap[2]:
                        new_route[swap[0]], new_route[swap[1]] = new_route[swap[1]], new_route[swap[0]]

                # new route of particle is applied and cost and pbest are changed if needed
                particle.route = new_route
                particle.update_costs_and_pbest()


if __name__ == "__main__":
    # enter 100,500,1000 to use one of the three provided datasets
    chosen_input = 100
    cities = read_cities(chosen_input)
    # the recommended settings without greedy route. if greedy_route is used you can decrease iterations and population_size
    pso = PSO(iterations=1000, population_size=2000, gbest_probability=0.05, pbest_probability=0.6, cities=cities)
    pso.run()
    # print gbest cost
    print(f'cost: {pso.gbest.pbest_cost}\t')
    # write iterations data to a file
    print_iteration_data(pso,chosen_input)
    # plot gbest route
    x_list, y_list = [], []
    for city in pso.gbest.pbest:
        x_list.append(city.x)
        y_list.append(city.y)
    x_list.append(pso.gbest.pbest[0].x)
    y_list.append(pso.gbest.pbest[0].y)
    fig = plt.figure(1)
    fig.suptitle('pso TSP')

    plt.plot(x_list, y_list, 'ro')
    plt.plot(x_list, y_list)
    plt.show(block=True)
