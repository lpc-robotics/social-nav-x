import math
import pytest
from arena_multi_hunav.timeline import Timeline, Sample

def test_interpolation_and_wrapped_yaw():
    t=Timeline(); t.append(Sample(0,0,0,math.radians(179),0,0,0),1)
    t.append(Sample(100,2,4,math.radians(-179),2,4,0),2)
    s=t.at(50); assert s.x==1 and s.y==2 and s.vx==1
    assert abs(abs(s.yaw)-math.pi)<1e-10
    with pytest.raises(ValueError): t.at(101)

def test_duplicate_does_not_refresh_watchdog_and_rewind_fails():
    t=Timeline(); sample=Sample(1,0,0,0,0,0,0); t.append(sample,1); t.append(sample,100)
    assert t.wall_seen==1
    with pytest.raises(ValueError): t.append(Sample(0,0,0,0,0,0,0),101)
