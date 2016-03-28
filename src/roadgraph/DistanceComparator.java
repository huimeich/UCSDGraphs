package roadgraph;

import java.util.Comparator;

public class DistanceComparator implements Comparator<MapGraphNode>{
	@Override
	public int compare(MapGraphNode x, MapGraphNode y) {
		if (x.getDistance() < y.getDistance()) {
			return -1;
		}
		if (x.getDistance() > y.getDistance()) {
			return 1;
		}
		return 0;
	}
}
