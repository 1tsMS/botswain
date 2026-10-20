// Botswain robot blocks and the editor. The program is saved as Blockly JSON and run by Python
// (botswain/control/program.py, which interprets every block itself); no code is generated or run in this page.
(function () {
  const MOVE = 210, TOOL = 20, STATE = 280, FLOW = 120, START = 45, LOG = 160;
  window.botswainPoints = [];

  const stmt = { previousStatement: null, nextStatement: null };
  const num = (name, value) => ({ type: 'field_number', name: name, value: value });
  const slot = (name) => ({ type: 'input_value', name: name, check: 'Number' });
  const shadowNum = (n) => ({ shadow: { type: 'math_number', fields: { NUM: n } } });

  Blockly.common.defineBlocksWithJsonArray([
    { type: 'botswain_start', message0: 'when Run is pressed %1 %2',
      args0: [{ type: 'input_dummy' }, { type: 'input_statement', name: 'DO' }], colour: START,
      tooltip: 'The program: only blocks inside this one run.' },
    // ---- motion ----
    Object.assign({ type: 'botswain_move_relative', message0: 'move %1 by %2 mm / deg',
      args0: [{ type: 'field_dropdown', name: 'AXIS', options: ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'].map((a) => [a, a]) },
              slot('DIST')],
      inputsInline: true, colour: MOVE, tooltip: 'Step relative to where the arm is, in the base frame.' }, stmt),
    Object.assign({ type: 'botswain_move_joints', message0: 'move joints J1 %1 J2 %2 J3 %3 J4 %4 J5 %5 J6 %6',
      args0: [num('J1', 0), num('J2', 0), num('J3', 90), num('J4', 0), num('J5', -90), num('J6', 0)],
      colour: MOVE, inputsInline: true, tooltip: 'Joint move to these angles (deg).' }, stmt),
    Object.assign({ type: 'botswain_move_pose', message0: '%1 to X %2 Y %3 Z %4 Rx %5 Ry %6 Rz %7',
      args0: [{ type: 'field_dropdown', name: 'KIND', options: [['move in a line', 'linear'], ['move (joint)', 'joint']] },
              num('X', -470), num('Y', -140), num('Z', 470), num('RX', 180), num('RY', 0), num('RZ', 90)],
      colour: MOVE, inputsInline: true, tooltip: 'Move the tool to this pose (mm / deg).' }, stmt),
    Object.assign({ type: 'botswain_home', message0: 'go home', colour: MOVE,
      tooltip: 'Joint move to the home pose in the robot config (only when confirmed).' }, stmt),
    Object.assign({ type: 'botswain_speed', message0: 'set speed %1 %', args0: [slot('PCT')], inputsInline: true,
      colour: MOVE, tooltip: 'Speed control, 1..100 % (the safety gate caps it).' }, stmt),
    // ---- gripper ----
    Object.assign({ type: 'botswain_gripper', message0: 'gripper %1',
      args0: [{ type: 'field_dropdown', name: 'STATE', options: [['open', 'open'], ['close', 'close']] }],
      colour: TOOL }, stmt),
    Object.assign({ type: 'botswain_gripper_force', message0: 'gripper close force %1 %', args0: [slot('PCT')],
      inputsInline: true, colour: TOOL, tooltip: 'Closing force for the next closes (DH: 20..100 %).' }, stmt),
    { type: 'botswain_gripper_caught', message0: 'gripper holds an object', output: 'Boolean', colour: TOOL,
      tooltip: 'True when the gripper reports "object caught".' },
    { type: 'botswain_gripper_opening', message0: 'gripper opening %', output: 'Number', colour: TOOL,
      tooltip: 'Measured opening, 0 closed .. 100 open.' },
    // ---- robot state ----
    { type: 'botswain_tcp', message0: 'tool %1', output: 'Number', colour: STATE,
      args0: [{ type: 'field_dropdown', name: 'AXIS', options: ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'].map((a) => [a, a]) }],
      tooltip: 'Where the tool is now (mm / deg, base frame).' },
    { type: 'botswain_joint', message0: 'joint %1', output: 'Number', colour: STATE,
      args0: [{ type: 'field_dropdown', name: 'JOINT', options: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'].map((a) => [a, a]) }],
      tooltip: 'Joint angle now (deg).' },
    // ---- wait and log ----
    Object.assign({ type: 'botswain_wait', message0: 'wait %1 s', args0: [slot('SECONDS')], inputsInline: true,
      colour: FLOW }, stmt),
    Object.assign({ type: 'botswain_log', message0: 'log %1', args0: [{ type: 'input_value', name: 'TEXT' }],
      inputsInline: true, colour: LOG, tooltip: 'Write a line in the Log dock.' }, stmt),
  ]);

  Blockly.Blocks['botswain_move_point'] = {
    init: function () {
      this.appendDummyInput()
        .appendField(new Blockly.FieldDropdown([['move (joint) to point', 'joint'], ['move in a line to point', 'linear']]), 'KIND')
        .appendField(new Blockly.FieldDropdown(function () {
          const p = window.botswainPoints;
          return p.length ? p.map((n) => [n, n]) : [['(no saved points)', '']];
        }), 'POINT');
      this.setPreviousStatement(true);
      this.setNextStatement(true);
      this.setColour(MOVE);
      this.setTooltip('Go to a saved point (Points page): joint move to its joints, or a straight line to its pose.');
    },
  };

  const block = (type, inputs) => (inputs ? { kind: 'block', type: type, inputs: inputs } : { kind: 'block', type: type });
  const toolbox = {
    kind: 'categoryToolbox',
    contents: [
      { kind: 'category', name: 'Program', colour: START, contents: [block('botswain_start')] },
      { kind: 'category', name: 'Motion', colour: MOVE, contents: [
        block('botswain_move_point'), block('botswain_move_relative', { DIST: shadowNum(10) }),
        block('botswain_move_pose'), block('botswain_move_joints'), block('botswain_home'),
        block('botswain_speed', { PCT: shadowNum(10) })] },
      { kind: 'category', name: 'Gripper', colour: TOOL, contents: [
        block('botswain_gripper'), block('botswain_gripper_force', { PCT: shadowNum(50) }),
        block('botswain_gripper_caught'), block('botswain_gripper_opening')] },
      { kind: 'category', name: 'Robot state', colour: STATE, contents: [block('botswain_tcp'), block('botswain_joint')] },
      { kind: 'category', name: 'Logic', colour: 210, contents: [
        block('controls_if'), block('logic_compare'), block('logic_operation'), block('logic_negate'),
        block('logic_boolean')] },
      { kind: 'category', name: 'Loops', colour: FLOW, contents: [
        block('controls_repeat_ext', { TIMES: shadowNum(3) }), block('controls_whileUntil'),
        block('controls_for', { FROM: shadowNum(1), TO: shadowNum(5), BY: shadowNum(1) })] },
      { kind: 'category', name: 'Math', colour: 230, contents: [
        block('math_number'), block('math_arithmetic', { A: shadowNum(1), B: shadowNum(1) })] },
      { kind: 'category', name: 'Variables', colour: 330, custom: 'VARIABLE' },
      { kind: 'category', name: 'Wait & log', colour: FLOW, contents: [
        block('botswain_wait', { SECONDS: shadowNum(1) }),
        block('botswain_log', { TEXT: { shadow: { type: 'text', fields: { TEXT: 'hello' } } } })] },
    ],
  };

  const starter = { blocks: { languageVersion: 0, blocks: [{ type: 'botswain_start', x: 30, y: 30,
    inputs: { DO: { block: { type: 'botswain_speed', inputs: { PCT: shadowNum(10) },
      next: { block: { type: 'botswain_move_relative', fields: { AXIS: 'Z' }, inputs: { DIST: shadowNum(20) },
        next: { block: { type: 'botswain_wait', inputs: { SECONDS: shadowNum(1) },
          next: { block: { type: 'botswain_move_relative', fields: { AXIS: 'Z' }, inputs: { DIST: shadowNum(-20) } } } } } } } } } } }] } };

  // Programs saved before these blocks took values have the number as a field: move it into the slot.
  const MOVED_TO_SLOT = { botswain_move_relative: ['DIST'], botswain_wait: ['SECONDS'], botswain_speed: ['PCT'] };
  function migrate(b) {
    if (!b || typeof b !== 'object') return;
    (MOVED_TO_SLOT[b.type] || []).forEach((name) => {
      if (b.fields && name in b.fields) {
        b.inputs = b.inputs || {};
        b.inputs[name] = shadowNum(Number(b.fields[name]));
        delete b.fields[name];
      }
    });
    Object.values(b.inputs || {}).forEach((i) => { migrate(i.block); migrate(i.shadow); });
    if (b.next) migrate(b.next.block);
  }

  window.botswainInit = function () {
    const ws = Blockly.inject('ws', { toolbox: toolbox, trashcan: true, sounds: false, media: 'media/', zoom: { controls: true, wheel: true },
                                      grid: { spacing: 24, length: 2, colour: '#d9d9d5', snap: true } });
    Blockly.serialization.workspaces.load(starter, ws);
    window.botswain = {
      save: () => JSON.stringify(Blockly.serialization.workspaces.save(ws)),
      load: (text) => {
        const program = JSON.parse(text);
        ((program.blocks || {}).blocks || []).forEach(migrate);
        ws.clear();
        Blockly.serialization.workspaces.load(program, ws);
        return true;
      },
      clear: () => { ws.clear(); Blockly.serialization.workspaces.load(starter, ws); return true; },
      highlight: (id) => { ws.highlightBlock(id || null); return true; },
      setPoints: (names) => { window.botswainPoints = names; return true; },
      ready: true,
    };
  };
})();
