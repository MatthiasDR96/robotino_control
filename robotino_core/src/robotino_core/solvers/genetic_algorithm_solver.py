import operator
from random import *

import numpy as np
import pandas as pd


def genetic_algorithm(objective_function, nvars, algorithm_params):
    
    # Initialize population
    npop = algorithm_params['population_size']
    pop = initial_population(nvars, npop)

    # Execute GA
    generations = algorithm_params['max_num_iteration']
    max_stall_generations = algorithm_params['max_stall_generations']
    elite_size = algorithm_params['elit_size']
    mutation_rate = algorithm_params['mutation_probability']

    # Loop over generations
    stall_gen = 0
    prev_best_fitness = 0
    for i in range(0, generations):
        pop, best_fitness = next_generation(objective_function, pop, elite_size, mutation_rate)
        if best_fitness == prev_best_fitness:
            stall_gen += 1
        prev_best_fitness = best_fitness
        if stall_gen == max_stall_generations:
            break

    # Get end solution
    pop_ranked = rank_routes(objective_function, pop)
    best_route_index = pop_ranked[0][0]
    best_fitness = pop_ranked[0][1]
    best_route = pop[best_route_index]
    return best_route, best_fitness


def initial_population(dim, npop):
    # For optimal resource management
    if type(dim) == int:
        population = []
        for i in range(npop):
            population.append(np.random.randint(2, size=dim))
    # For heuristic solution
    else:
        population = []
        for i in range(npop - dim[0]):
            indices = np.random.randint(dim[0], size=dim[1])
            pop = np.zeros(dim)
            for i in range(dim[1]):
                pop[indices[i], i] = 1
            population.append(np.resize(pop, (dim[0] * dim[1],)))
    return population


def rank_routes(fitness, population):
    fitness_results = {}
    for i in range(0, len(population)):
        fitness_results[i] = fitness(population[i])
    sorted_pop = sorted(fitness_results.items(), key=operator.itemgetter(1), reverse=False)
    return sorted_pop


def selection(pop_ranked, elite_size):
    selection_results = []
    df = pd.DataFrame(np.array(pop_ranked), columns=["Index", "Fitness"])
    df['cum_sum'] = df.Fitness.cumsum()
    df['cum_perc'] = 100 * df.cum_sum / df.Fitness.sum()
    for i in range(elite_size):
        selection_results.append(pop_ranked[i][0])
    for i in range(len(pop_ranked) - elite_size):
        pick = randint(0, len(pop_ranked[0]) - 1)
        selection_results.append(pop_ranked[pick][0])
    return selection_results


def mating_pool(population, selection_results):
    matingpool = []
    for i in range(len(selection_results)):
        index = selection_results[i]
        matingpool.append(population[index])
    return matingpool


def breed(parent1, parent2):
    child = np.zeros_like(parent1)
    gene_a = int(random() * len(parent1))
    gene_b = int(random() * len(parent1))
    start_gene = min(gene_a, gene_b)
    end_gene = max(gene_a, gene_b)
    child[start_gene:end_gene] = parent1[start_gene:end_gene]
    child[end_gene:] = parent2[end_gene:]
    child[:start_gene] = parent2[:start_gene]
    return list(child)


def breed_population(matingpool, elite_size):
    children = []
    length = len(matingpool) - elite_size
    pool = sample(matingpool, len(matingpool))
    for i in range(elite_size):
        children.append(matingpool[i])
    for i in range(length):
        child = breed(pool[i], pool[len(matingpool) - i - 1])
        children.append(child)
    return children


def mutate(individual, mutation_rate):
    for swapped in range(len(individual)):
        if random() < mutation_rate:
            swap_with = int(random() * len(individual))
            city1 = individual[swapped]
            city2 = individual[swap_with]
            individual[swapped] = city2
            individual[swap_with] = city1
        return individual


def mutate_population(population, mutation_rate):
    mutated_pop = []
    for ind in range(len(population)):
        mutated_ind = mutate(population[ind], mutation_rate)
        mutated_pop.append(mutated_ind)
    return mutated_pop


def next_generation(fitness, current_gen, elite_size, mutation_rate):
    pop_ranked = rank_routes(fitness, current_gen)
    selection_results = selection(pop_ranked, elite_size)
    best_fitness = pop_ranked[0][1]
    matingpool = mating_pool(current_gen, selection_results)
    children = breed_population(matingpool, elite_size)
    next_generation = mutate_population(children, mutation_rate)
    return next_generation, best_fitness
