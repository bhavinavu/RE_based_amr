#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
from amr_drl_navigation import periodic_ops


def update_target_variables(target_variables,
                            source_variables,
                            tau=1.0,
                            use_locking=False,
                            name="update_target_variables"):
    """
    Returns an op to update a list of target variables from source variables.

    The update rule is:
    `target_variable = (1 - tau) * target_variable + tau * source_variable`.

    :param target_variables: a list of the variables to be updated.
    :param source_variables: a list of the variables used for the update.
    :param tau: weight used to gate the update. The permitted range is 0 < tau <= 1,
        with small tau representing an incremental update, and tau == 1
        representing a full update (that is, a straight copy).
    :param use_locking: use `tf.Variable.assign`'s locking option when assigning
        source variable values to target variables.
    :param name: sets the `name_scope` for this op.
    :raise TypeError: when tau is not a Python float
    :raise ValueError: when tau is out of range, or the source and target variables
        have different numbers or shapes.
    :return: An op that executes all the variable updates.
    """
    if not isinstance(tau, float):
        raise TypeError("Tau has wrong type (should be float) {}".format(tau))
    if not 0.0 < tau <= 1.0:
        raise ValueError("Invalid parameter tau {}".format(tau))
    if len(target_variables) != len(source_variables):
        raise ValueError("Number of target variables {} is not the same as "
                         "number of source variables {}".format(
                             len(target_variables), len(source_variables)))
    
    def _shape_to_tuple(v):
        # try common attributes in order
        shp = None
        if hasattr(v, "get_shape"):            # TF1 TensorShape or Variable
            try:
                shp = v.get_shape()
            except Exception:
                shp = None
        if shp is None and hasattr(v, "shape"):
            shp = v.shape
        if shp is None and hasattr(v, "_shape"):
            shp = v._shape
        # convert to tuple (handles TensorShape and eager shapes)
        try:
            return tuple(shp.as_list()) if hasattr(shp, "as_list") else tuple(shp)
        except Exception:
            # last resort: try casting to tuple directly
            return tuple(shp)

    same_shape = all(_shape_to_tuple(trg) == _shape_to_tuple(src)
                     for trg, src in zip(target_variables, source_variables))
    if not same_shape:
        raise ValueError("Target variables don't have the same shape as source "
                         "variables.")
    
    def _safe_assign(var, value):
        try:
            # try the keyword form first (works on many TF2 builds)
            return var.assign(value, use_locking=use_locking)
        except TypeError:
            try:
                # some builds accept positional but we've already seen that break;
                # fallback to single-arg assign
                return var.assign(value)
            except Exception as e:
                # re-raise with more context
                raise RuntimeError(f"Failed to assign variable {var}: {e}") from e

    def update_op(target_variable, source_variable, tau):
        if tau == 1.0:
            return _safe_assign(target_variable, source_variable)
        else:
            new_val = tau * source_variable + (1.0 - tau) * target_variable
            return _safe_assign(target_variable, new_val)

    # with tf.name_scope(name, values=target_variables + source_variables):
    update_ops = [update_op(target_var, source_var, tau)
                  for target_var, source_var
                  in zip(target_variables, source_variables)]
    return tf.group(name="update_all_variables", *update_ops)


def periodic_target_update(target_variables,
                           source_variables,
                           update_period,
                           tau=1.0,
                           use_locking=False,
                           name="periodic_target_update"):
    """
    Returns an op to periodically update a list of target variables.

    The `update_target_variables` op is executed every `update_period`
    executions of the `periodic_target_update` op.

    The update rule is:
    `target_variable = (1 - tau) * target_variable + tau * source_variable`.

    :param target_variables: a list of the variables to be updated.
    :param source_variables: a list of the variables used for the update.
    :param update_period: inverse frequency with which to apply the update.
    :param tau: weight used to gate the update. The permitted range is 0 < tau <= 1,
        with small tau representing an incremental update, and tau == 1
        representing a full update (that is, a straight copy).
    :param use_locking: use `tf.variable.Assign`'s locking option when assigning
        source variable values to target variables.
    :param name: sets the `name_scope` for this op.
    :return: An op that periodically updates `target_variables` with `source_variables`.
    """

    def update_op():
        return update_target_variables(
            target_variables, source_variables, tau, use_locking)

    with tf.name_scope(name, values=target_variables + source_variables):
        return periodic_ops.periodically(update_op, update_period)
