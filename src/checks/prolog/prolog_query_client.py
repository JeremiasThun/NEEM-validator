#!/usr/bin/env python

# the following code is a modified version of the RosprologRestClient:
# https://github.com/knowrob/rosprolog/blob/master/scripts/RosprologRestClient.py
# Last accessed: commit c979f70f7ddeb1bc1b6b6b371e3e1bc38779f91e

import rospy
from json_prolog_msgs.srv import PrologNextSolution, PrologNextSolutionResponse, PrologFinish
from json_prolog_msgs.srv import PrologQuery as PrologQueryMessage
import json

class PrologQuery(object):
	def __init__(self, namespace='rosprolog', timeout=None, wait_for_services=True):
		"""
		@type 	namespace: str
		@param 	timeout: Amount of time in seconds spend waiting for rosprolog to become available.
		@type 	timeout: int
		"""
		self.id = 0
		self.opened_queries_left = False
		self._simple_query_srv = rospy.ServiceProxy(
			'{}/query'.format(namespace), PrologQueryMessage)
		self._next_solution_srv = rospy.ServiceProxy(
			'{}/next_solution'.format(namespace), PrologNextSolution)
		self._finish_query_srv = rospy.ServiceProxy(
			'{}/finish'.format(namespace), PrologFinish)
		if wait_for_services:
			rospy.loginfo('waiting for {} services'.format(namespace))
			self._finish_query_srv.wait_for_service(timeout=timeout)
			self._simple_query_srv.wait_for_service(timeout=timeout)
			self._next_solution_srv.wait_for_service(timeout=timeout)
			rospy.loginfo('{} services ready'.format(namespace))

	def post_query(self, query):
		"""
		@param	query: the query string
		@type	query: str
		"""
		if (self.opened_queries_left):
			self.finish_query()
		self.id += 1
		self.last_query = query
		result = self._simple_query_srv(id=str(self.id), query=query)
		self.opened_queries_left = True
		if result.ok:
			return True
		else:
			self.id -= 1
			return False

	def get_solutions(self, max_solution_count):
		"""
		@param	solution_count: the query string
		@type	solution_count: int
		"""
		solutions = []
		all_solutions = False
		if max_solution_count == -1:
			all_solutions = True
		try:
			while(all_solutions or (len(solutions) < max_solution_count)):
				try:
					next_solution = self.get_next_solution()
					solutions.append(next_solution)
				except StopIteration:
					return solutions
				#next_solution = self._next_solution_srv(id=str(self.id))
				#if next_solution.status == PrologNextSolutionResponse.OK:
				#	solutions.append(dict(json.loads(next_solution.solution)))
				#elif next_solution.status == PrologNextSolutionResponse.NO_SOLUTION:
				#	break
				#else:
				#	raise ValueError('Bad query') # todo: not the best exception
		finally:
			self.finish_query()
		return solutions

	def get_next_solution(self):
		"""
		Returns the next solution.
		"""
		try:
			next_solution = self._next_solution_srv(id=str(self.id))
		except:
			finish_query()
			raise
		if next_solution.status == PrologNextSolutionResponse.OK:
			return dict(json.loads(next_solution.solution))
		elif next_solution.status == PrologNextSolutionResponse.NO_SOLUTION:
			self.finish_query()
			raise StopIteration('No more solutions')
		else:
			raise ValueError('Prolog query couldn\'t get executed: "' + self.last_query + '". Returned status: ' + str(next_solution.status))

	def finish_query(self):
		self.opened_queries_left = False
		self._finish_query_srv(id=str(self.id))
