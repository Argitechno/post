instruction_sets = {}

def register_instruction_set(key):
    def decorator(func_or_class):
        instruction_sets[key] = func_or_class
        return func_or_class
    return decorator

#Import to register.
from post_instruction_sets.dynamic_loop import dynamic_loop_instruction_set
