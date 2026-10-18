// Botswain robot blocks and the editor. The program is saved as Blockly JSON and run by Python
// (botswain/control/program.py); no code is generated or executed in this page.
(function () {
  const MOVE = 210, TOOL = 20, FLOW = 120, START = 45;
  window.botswainPoints = [];

  const num = (name, value, extra) => Object.assign({ type: 'field_number', name: name, value: value }, extra || {});
  const stmt = { previousStatement: null, nextStatement: null };

  Blockly.common.defineBlocksWithJsonArray([
    { type: 'botswain_start', message0: 'when Run is pressed %1 %2',
      args0: [{ type: 'input_dummy' }, { type: 'input_statement', name: 'DO' }], colour: START,
      tooltip: 'The program: only blocks inside this one run.' },
    Object.assign({ type: 'botswain_move_relative', message0: 'move %1 by %2 mm / deg',
      args0: [{ type: 'field_dropdown', name: 'AXIS',
                options: ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'].map((a) => [a, a]) }, num('DIST', 10)],
      colour: MOVE, tooltip: 'Step relative to where the arm is, in the base frame.' }, stmt),
    Object.assign({ type: 'botswain_move_joints',
      message0: 'move joints J1 %1 J2 %2 J3 %3 J4 %4 J5 %5 J6 %6',
      args0: [num('J1', 0), num('J2', 0), num('J3', 90), num('J4', 0), num('J5', -90), num('J6', 0)],
      colour: MOVE, inputsInline: true, tooltip: 'Joint move to these angles (deg).' }, stmt),
    Object.assign({ type: 'botswain_move_pose', message0: '%1 to X %2 Y %3 Z %4 Rx %5 Ry %6 Rz %7',
      args0: [{ type: 'field_dropdown', name: 'KIND', options: [['move in a line', 'linear'], ['move (joint)', 'joint']] },
              num('X', -470), num('Y', -140), num('Z', 470), num('RX', 180), num('RY', 0), num('RZ', 90)],
      colour: MOVE, inputsInline: true, tooltip: 'Move the tool to this pose (mm / deg).' }, stmt),
    Object.assign({ type: 'botswain_home', message0: 'go home', colour: MOVE,
      tooltip: 'Joint move to the home pose in the robot YAML (only when confirmed).' }, stmt),
    Object.assign({ type: 'botswain_gripper', message0: 'gripper %1',
      args0: [{ type: 'field_dropdown', name: 'STATE', options: [['open', 'open'], ['close', 'close']] }],
      colour: TOOL }, stmt),
    Object.assign({ type: 'botswain_speed', message0: 'set speed %1 %',
      args0: [num('PCT', 10, { min: 1, max: 100, precision: 1 })], colour: TOOL }, stmt),
    Object.assign({ type: 'botswain_wait', message0: 'wait %1 s',
      args0: [num('SECONDS', 1, { min: 0 })], colour: FLOW }, stmt),
  ]);

  Blockly.Blocks['botswain_move_point'] = {
    init: function () {
      this.appendDummyInput().appendField('move to point').appendField(new Blockly.FieldDropdown(function () {
        const p = window.botswainPoints;
        return p.length ? p.map((n) => [n, n]) : [['(no saved points)', '']];
      }), 'POINT');
      this.setPreviousStatement(true);
      this.setNextStatement(true);
      this.setColour(MOVE);
      this.setTooltip('Joint move to a saved point (Points page).');
    },
  };

  const toolbox = {
    kind: 'categoryToolbox',
    contents: [
      { kind: 'category', name: 'Program', colour: START, contents: [{ kind: 'block', type: 'botswain_start' }] },
      { kind: 'category', name: 'Move', colour: MOVE, contents: [
        { kind: 'block', type: 'botswain_move_point' }, { kind: 'block', type: 'botswain_move_relative' },
        { kind: 'block', type: 'botswain_move_pose' }, { kind: 'block', type: 'botswain_move_joints' },
        { kind: 'block', type: 'botswain_home' }] },
      { kind: 'category', name: 'Tool', colour: TOOL, contents: [
        { kind: 'block', type: 'botswain_gripper' }, { kind: 'block', type: 'botswain_speed' }] },
      { kind: 'category', name: 'Flow', colour: FLOW, contents: [
        { kind: 'block', type: 'controls_repeat_ext',
          inputs: { TIMES: { shadow: { type: 'math_number', fields: { NUM: 3 } } } } },
        { kind: 'block', type: 'botswain_wait' }] },
    ],
  };

  const starter = { blocks: { languageVersion: 0, blocks: [{ type: 'botswain_start', x: 30, y: 30,
    inputs: { DO: { block: { type: 'botswain_speed', fields: { PCT: 10 },
      next: { block: { type: 'botswain_move_relative', fields: { AXIS: 'Z', DIST: 20 },
        next: { block: { type: 'botswain_wait', fields: { SECONDS: 1 },
          next: { block: { type: 'botswain_move_relative', fields: { AXIS: 'Z', DIST: -20 } } } } } } } } } } }] } };

  window.botswainInit = function () {
    const ws = Blockly.inject('ws', { toolbox: toolbox, trashcan: true, sounds: false, media: 'media/', zoom: { controls: true, wheel: true },
                                      grid: { spacing: 24, length: 2, colour: '#d9d9d5', snap: true } });
    Blockly.serialization.workspaces.load(starter, ws);
    window.botswain = {
      save: () => JSON.stringify(Blockly.serialization.workspaces.save(ws)),
      load: (text) => { ws.clear(); Blockly.serialization.workspaces.load(JSON.parse(text), ws); return true; },
      clear: () => { ws.clear(); Blockly.serialization.workspaces.load(starter, ws); return true; },
      highlight: (id) => { ws.highlightBlock(id || null); return true; },
      setPoints: (names) => { window.botswainPoints = names; return true; },
      ready: true,
    };
  };
})();
