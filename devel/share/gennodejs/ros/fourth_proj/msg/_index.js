
"use strict";

let sensmsg = require('./sensmsg.js');
let cmdmsg = require('./cmdmsg.js');
let delayactionActionResult = require('./delayactionActionResult.js');
let delayactionResult = require('./delayactionResult.js');
let delayactionAction = require('./delayactionAction.js');
let delayactionActionGoal = require('./delayactionActionGoal.js');
let delayactionActionFeedback = require('./delayactionActionFeedback.js');
let delayactionFeedback = require('./delayactionFeedback.js');
let delayactionGoal = require('./delayactionGoal.js');

module.exports = {
  sensmsg: sensmsg,
  cmdmsg: cmdmsg,
  delayactionActionResult: delayactionActionResult,
  delayactionResult: delayactionResult,
  delayactionAction: delayactionAction,
  delayactionActionGoal: delayactionActionGoal,
  delayactionActionFeedback: delayactionActionFeedback,
  delayactionFeedback: delayactionFeedback,
  delayactionGoal: delayactionGoal,
};
