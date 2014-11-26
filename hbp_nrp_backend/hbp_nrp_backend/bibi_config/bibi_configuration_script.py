"""
Script to run CLE from BIBI Configuration File
"""
from hbp_nrp_backend.bibi_config.generated import generated_bibi_api

__author__ = 'GeorgHinkel'

import jinja2
import os

__device_types = {'ACSource': 'ac_source', 'DCSource': 'dc_source',
                  'FixedFrequency': 'fixed_frequency',
                  'LeakyIntegratorAlpha': 'leaky_integrator_alpha',
                  'LeakyIntegratorExp': 'leaky_integrator_exp',
                  'NCSource': 'nc_source',
                  'Poisson': 'poisson'}
__device_properties = {'ACSource': 'amplitude', 'DCSource': 'amplitude',
                       'FixedFrequency': 'rate',
                       'LeakyIntegratorAlpha': 'voltage',
                       'LeakyIntegratorExp': 'voltage',
                       'NCSource': 'mean',
                       'Poisson': 'rate'}
__operator_symbols = {generated_bibi_api.Subtract: '({0} - {1})',
                      generated_bibi_api.Add: '({0} + {1})',
                      generated_bibi_api.Multiply: '{0} * {1}',
                      generated_bibi_api.Divide: '{0} / {1}',
                      generated_bibi_api.Min: 'min({0}, {1})',
                      generated_bibi_api.Max: 'max({0}, {1})'}


def remove_extension(fname):
    """
    Removes the extension from the given file name
    :param fname: The file name
    """
    return os.path.splitext(fname)[0]


def get_device_name(device_type):
    """
    Gets the CLE name of the given device type
    :param device_type: The device type
    """
    return __device_types[device_type]


def print_expression(expression):
    """
    Prints the given flow expression to a string
    :param expression: The expression to be printed
    """
    if isinstance(expression, generated_bibi_api.Scale):
        return str(expression.factor) + ' * ' + print_expression(expression.inner)
    if isinstance(expression, generated_bibi_api.Call):
        temp = expression.type_ + '('
        i = 1
        for argument in expression.argument:
            temp += argument.name + '=' + print_expression(argument.value)
            if i < len(expression.argument):
                temp += ', '
            i += 1
        return temp + ')'
    if isinstance(expression, generated_bibi_api.Operator):
        return print_operator(expression)
    if isinstance(expression, generated_bibi_api.ArgumentReference):
        if expression.property is None:
            return expression.name
        else:
            return expression.name + '.' + expression.property
    if isinstance(expression, generated_bibi_api.Constant):
        return str(expression.value)
    raise Exception('No idea how to print expression of type ' + type(expression))


def print_operator(expression):
    """
    Prints the given operator expression to a string
    :param expression: The operator expression to be printed
    """
    text = print_expression(expression.operand[0])
    operator = __operator_symbols[type(expression)]
    for i in range(1, len(expression.operand)):
        text = operator.format(text, print_expression(expression.operand[i]))
    return text


def get_default_property(device_type):
    """
    Gets the default property for the given device type
    :param device_type: The device type
    """
    return __device_properties[device_type]


def get_neurons(device):
    """
    Gets a string representing the accessed neuron population
    :param device: The device
    """
    neurons = device.neurons
    if isinstance(neurons, generated_bibi_api.Index):
        return neurons.population + '[' + str(neurons.index) + ']'
    if isinstance(neurons, generated_bibi_api.Range):
        return neurons.population + '[' + str(neurons.from_) + ',' + str(neurons.to) + ']'
    raise Exception("Dont know how to process neuron selector " + device.extensiontype_)


def compute_dependencies(config):
    """
    Computed the dependencies of the given configuration
    :param config: The BIBI configuration
    """
    dependencies = set()
    for tf in config.transferFunction:
        for local in tf.local:
            __add_dependencies_for_expression(local.body, dependencies)
        if isinstance(tf, generated_bibi_api.Neuron2Robot):
            dependencies.add(tf.topic.type_)
        else:
            for topic in tf.topic:
                dependencies.add(topic.type_)
    return dependencies


def __add_dependencies_for_expression(expression, dependencies):
    """
    Adds the dependencies for the given expression to the given set of dependencies
    :param expression: The expression that may cause dependencies
    :param dependencies: The dependencies detected so far
    """
    if isinstance(expression, generated_bibi_api.Scale):
        __add_dependencies_for_expression(expression.inner, dependencies)
    if isinstance(expression, generated_bibi_api.Call):
        dependencies.add(expression.type_)
        for argument in expression.argument:
            __add_dependencies_for_expression(argument.value, dependencies)
    if isinstance(expression, generated_bibi_api.Operator):
        for operand in expression.operand:
            __add_dependencies_for_expression(operand, dependencies)


def generate_cle(bibi_conf, script_file_name):
    """
    Generates Code to run the CLE based on the given configuration file
    :param bibi_conf: The BIBI configuration
    :param script_file_name: The file name of the script to be generated
    """
    templatePath = os.path.join(os.path.split(__file__)[0], 'cle_template.pyt')
    templateFile = open(templatePath, 'r')
    template = jinja2.Template(templateFile.read())
    templateFile.close()

    config = generated_bibi_api.parse(bibi_conf, silence=True)
    names = dict(globals())
    names['config'] = config
    names['dependencies'] = compute_dependencies(config)
    # system functions are somehow not included in globals
    names['len'] = len
    outputFile = open(script_file_name, 'w')
    outputFile.write(template.render(names))
    outputFile.close()
