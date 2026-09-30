import copy
import pytest
from arena_multi_hunav.psychology import (
    Entity, MentalState, Modifiers, NoOpPsychology, Proposal, make_context, prepare_step,
)

def test_noop_transaction_and_replay():
    model = NoOpPsychology(); model.initialize({}, (1, 2), 11)
    previous = {1: MentalState({"fear": 0.2}, {"S": 1}), 2: MentalState()}
    context = make_context(100, .025, [], [])
    result = model.step(context, previous); result.validate(previous)
    assert result.states == previous
    result.states[1].variables["fear"] = 0.9
    assert previous[1].variables["fear"] == .2
    restored = NoOpPsychology(); restored.restore(model.snapshot())
    assert restored.step(context, previous) == model.step(context, previous)

def test_all_robot_stimuli_are_exposed():
    person = Entity(1, "person", 0, 0, 0, 0, .4)
    robots = [Entity(-1, "robot_1", 1, 0, .26, 0, .35), Entity(-2, "robot_2", 0, 2, 0, .26, .35)]
    ctx = make_context(0,.025,[person],robots)
    assert {n.target_id for n in ctx.neighbors} == {-1,-2}
    assert all(n.is_robot for n in ctx.neighbors)
    assert sorted(n.distance for n in ctx.neighbors) == [1,2]

@pytest.mark.parametrize("modifier", [Modifiers(speed_scale=float('nan')), Modifiers(space_scale=.9), Modifiers(robot_scale=-1)])
def test_bad_modifiers_fail(modifier):
    with pytest.raises(ValueError):
        Proposal({1: MentalState()}, {1: modifier}).validate([1])

def test_future_plugin_can_modulate_without_mutating_context():
    class Slower(NoOpPsychology):
        def step(self, context, previous_state):
            return Proposal(copy.deepcopy(previous_state), {i: Modifiers(speed_scale=.5) for i in self.agent_ids})
    plugin=Slower(); plugin.initialize({},(1,),11)
    proposal=plugin.step(make_context(0,.025,[],[]),{1:MentalState()})
    proposal.validate([1]); assert proposal.modifiers[1].speed_scale == .5


@pytest.mark.parametrize('failure',[None,'exception','invalid'])
def test_plugin_state_is_not_committed_before_motion_success(failure):
    class Stateful:
        counter=0
        def snapshot(self):return {'counter':self.counter}
        def restore(self,snapshot):self.counter=snapshot['counter']
        def step(self,context,previous):
            self.counter+=1;previous[1].variables['fear']=.9
            if failure=='exception':raise RuntimeError('plugin failed')
            return Proposal(previous,{1:Modifiers(speed_scale=-1 if failure=='invalid' else 1)})
    plugin=Stateful();previous={1:MentalState({'fear':.2})}
    if failure:
        with pytest.raises((RuntimeError,ValueError)):prepare_step(plugin,make_context(0,.025,[],[]),previous)
    else:
        proposal,snapshot=prepare_step(plugin,make_context(0,.025,[],[]),previous)
        assert snapshot=={'counter':1} and proposal.states[1].variables['fear']==.9
    assert plugin.counter==0 and previous[1].variables['fear']==.2
