
Blockly.Blocks['yolobit_ble_start'] = {
  init: function() {
    this.jsonInit(
      {"colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_START_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_START_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {
          "type": "input_value",
          "name": "NAME",
          "check": "String"
        }],
      "inputsInline": true,
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_START_HELPURL});
  }
};

Blockly.Blocks['yolobit_ble_connect'] = {
  init: function() {
    this.jsonInit(
      {"colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_CONNECT_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_CONNECT_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {
          "type": "input_value",
          "name": "NAME",
          "check": "String"
        }],
      "inputsInline": true,
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_CONNECT_HELPURL});
  }
};

Blockly.Blocks['yolobit_ble_repl'] = {
  init: function() {
    this.jsonInit(
      {"colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_REPL_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_REPL_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {
          "type": "input_value",
          "name": "NAME",
          "check": "String"
        }],
      "inputsInline": true,
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_REPL_HELPURL});
  }
};

Blockly.Blocks['yolobit_ble_send'] = {
  init: function() {
    this.jsonInit(
      {"colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_SEND_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_SEND_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {
          "type": "input_value",
          "name": "message",
        }],
      "inputsInline": true,
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_SEND_HELPURL});
  }
};

Blockly.Blocks['yolobit_ble_send_namevalue'] = {
  init: function() {
    this.jsonInit(
      {"colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_SEND_NAMEVALUE_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_SEND_NAMEVALUE_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {
          "type": "input_value",
          "name": "name",
        },
        {
          "type": "input_value",
          "name": "value",
        }],
      "inputsInline": true,
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_SEND_NAMEVALUE_HELPURL});
  }
};

Blockly.Blocks['yolobit_ble_on_receive_message'] = {
  init: function() {
    this.jsonInit({
      "colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_MESSAGE_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_MESSAGE_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {"variable": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_MESSAGE_MESSAGE1, "type": "field_variable", "name": "message"}, 
        {"type": "input_dummy"}, 
        {"type": "input_statement", "name": "ACTION"}
      ], 
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_MESSAGE_HELPURL
    });
  }
};

Blockly.Blocks['yolobit_ble_on_receive_number'] = {
  init: function() {
    this.jsonInit({
      "colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NUMBER_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NUMBER_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {"variable": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NUMBER_MESSAGE1, "type": "field_variable", "name": "number"}, 
        {"type": "input_dummy"}, 
        {"type": "input_statement", "name": "ACTION"}
      ], 
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NUMBER_HELPURL
    });
  }
};

Blockly.Blocks['yolobit_ble_on_receive_namevalue'] = {
  init: function() {
    this.jsonInit({
      "colour": "#005a9e", 
      "nextStatement": null, 
      "tooltip": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NAMEVALUE_TOOLTIP, 
      "message0": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NAMEVALUE_MESSAGE0, 
      "previousStatement": null, 
      "args0": [
        {"variable": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NAMEVALUE_MESSAGE1, "type": "field_variable", "name": "name"}, 
        {"variable": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NAMEVALUE_MESSAGE2, "type": "field_variable", "name": "value"}, 
        {"type": "input_dummy"}, 
        {"type": "input_statement", "name": "ACTION"}
      ], 
      "helpUrl": Blockly.Msg.YOLOBIT_BLE_ON_RECEIVE_NAMEVALUE_HELPURL
    });
  }
};

// Any imports need to be reserved words
Blockly.Python.addReservedWords('ble_start');
Blockly.Python.addReservedWords('ble_connect');
Blockly.Python.addReservedWords('ble_on_rx');
Blockly.Python.addReservedWords('ble_send');

Blockly.Python['yolobit_ble_start'] = function(block) {
  Blockly.Python.definitions_['import_ble_peripheral'] = 'from ble_uart_peripheral import *';
  var value_name = Blockly.Python.valueToCode(block, 'NAME', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'ble_start(' + value_name + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_connect'] = function(block) {
  Blockly.Python.definitions_['import_ble_central'] = 'from ble_simple_central import *';
  var value_name = Blockly.Python.valueToCode(block, 'NAME', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'ble_connect(' + value_name + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_repl'] = function(block) {
  Blockly.Python.definitions_['import_ble_repl'] = 'from ble_uart_repl import *';
  var value_name = Blockly.Python.valueToCode(block, 'NAME', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'ble_start_repl(' + value_name + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_send'] = function(block) {
  var value_msg = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'ble_send(' + value_msg + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_send_namevalue'] = function(block) {
  var value_name = Blockly.Python.valueToCode(block, 'name', Blockly.Python.ORDER_ATOMIC);
  var value_value = Blockly.Python.valueToCode(block, 'value', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'ble_send(' + value_name + ', ' + value_value + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_on_receive_message'] = function(block) {
  var variable_message = Blockly.Python.variableDB_.getName(block.getFieldValue('message'), Blockly.Variables.NAME_TYPE);
  var statements_action = Blockly.Python.statementToCode(block, 'ACTION');
  // TODO: Assemble Python into code variable.
  var cbFunctionName = Blockly.Python.provideFunction_(
    'ble_on_rx_callback',
    ['def ' + Blockly.Python.FUNCTION_NAME_PLACEHOLDER_ + '(' + variable_message + '):',
      statements_action || Blockly.Python.PASS
    ]);
  
  var code = 'ble_on_rx(' + cbFunctionName + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_on_receive_number'] = function(block) {
  var variable_number = Blockly.Python.variableDB_.getName(block.getFieldValue('number'), Blockly.Variables.NAME_TYPE);
  var statements_action = Blockly.Python.statementToCode(block, 'ACTION');
  // TODO: Assemble Python into code variable.
  var cbFunctionName = Blockly.Python.provideFunction_(
    'ble_on_rx_callback',
    ['def ' + Blockly.Python.FUNCTION_NAME_PLACEHOLDER_ + '(' + variable_number + '):',
      Blockly.Python.INDENT +  variable_number + ' = int(' + variable_number + ') if ' + variable_number + '.isdigit() else None\n' + 
      statements_action || Blockly.Python.PASS
    ]);
  
  var code = 'ble_on_rx(' + cbFunctionName + ')\n';
  return code;
};

Blockly.Python['yolobit_ble_on_receive_namevalue'] = function(block) {
  Blockly.Python.definitions_['import_json'] = 'import json';
  var variable_name = Blockly.Python.variableDB_.getName(block.getFieldValue('name'), Blockly.Variables.NAME_TYPE);
  var variable_value = Blockly.Python.variableDB_.getName(block.getFieldValue('value'), Blockly.Variables.NAME_TYPE);
  var statements_action = Blockly.Python.statementToCode(block, 'ACTION');
  // TODO: Assemble Python into code variable.
  var cbFunctionName = Blockly.Python.provideFunction_(
    'ble_on_rx_callback',
    ['def ' + Blockly.Python.FUNCTION_NAME_PLACEHOLDER_ + '(msg):',
      Blockly.Python.INDENT + variable_name + " = None\n" +
      Blockly.Python.INDENT + variable_value + " = None\n" +
      Blockly.Python.INDENT + "try:\n" +
      Blockly.Python.INDENT + Blockly.Python.INDENT + variable_name + ", " + variable_value + " = json.loads(msg)\n" +
      Blockly.Python.INDENT + "except:\n" + 
      Blockly.Python.INDENT + Blockly.Python.INDENT + "pass\n" +
      statements_action || Blockly.Python.PASS
    ]);
  
  var code = 'ble_on_rx(' + cbFunctionName + ')\n';
  return code;
};