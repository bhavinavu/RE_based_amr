#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf


def periodically(body, period, name="periodically"):
    """
    Periodically performs a tensorflow op.

    The body tensorflow op will be executed every `period` times the periodically
    op is executed. More specifically, with `n` the number of times the op has
    been executed, the body will be executed when `n` is a non zero positive
    multiple of `period` (i.e. there exist an integer `k > 0` such that
    `k * period == n`).

    If `period` is 0 or `None`, it would not perform any op and would return a
    `tf.no_op()`.

    :param body (callable): callable that returns the tensorflow op to be performed every time
        an internal counter is divisible by the period. The op must have no
        output (for example, a tf.group()).
    :param period (int): inverse frequency with which to perform the op.
    :param name (str): name of the variable_scope.
    :raise TypeError: if body is not a callable.
    :raise ValueError: if period is negative.
    :return: An op that periodically performs the specified op.
    """
    if not callable(body):
        raise TypeError("body must be callable.")

    if period is None or period == 0:
        return tf.no_op()

    if period < 0:
        raise ValueError("period cannot be less than 0.")

    if period == 1:
        return body()

    with tf.variable_scope(None, default_name=name):
        counter = tf.get_variable(
            "counter",
            shape=[],
            dtype=tf.int64,
            trainable=False,
            initializer=tf.constant_initializer(period, dtype=tf.int64))

        def _wrapped_body():
            with tf.control_dependencies([body()]):
                # Done the deed, resets the counter.
                return counter.assign(1)

        update = tf.cond(
            tf.equal(counter, period), _wrapped_body, lambda: counter.assign_add(1))

    return update
