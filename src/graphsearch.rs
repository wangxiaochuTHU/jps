use super::base::{JPS3DNeib, State};
// use float_ord::{sort, FloatOrd};
use im::Vector;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use priority_queue::PriorityQueue;
use std::cmp::Reverse;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Default)]
/// `GraphSearch` is the struct type for path finding.
///
/// `pq_` :  a Priorty Queue for saving possible nodes.
///
/// `hm_` :  a HashMap for saving nodes that have been visited.
///
/// `path_`: the path in the forward order, i.e. from start to goal.
///
/// `turnings` : the turning points along the path in the forward order, including start and goal.
/// `turnings` can be differenced for constructing lines, e.g.  line[k] := (turnings[k], turnings[k+1])
///
///  `ns_` : all neighbors in terms of relative position.
///
///  `jn3d_`: all jps neighbors.
///
///  `fmap_`: HashSet containing all free grids.
///
///  `omap` : HashSet containing all occupied grids.
///
pub struct GraphSearch {
    pub(crate) pq_: PriorityQueue<State, Reverse<OrderedFloat<f32>>>,

    pub(crate) hm_: HashMap<(i32, i32, i32), State>,

    /// the path in the forward order, i.e. from start to goal.
    pub path_: Vec<State>,

    /// the turning points along the path in the forward order, including start and goal.
    ///
    /// `turnings` can be differenced for constructing lines, e.g.  line[k] := (turnings[k], turnings[k+1])
    pub turnings: Vec<(i32, i32, i32)>,

    pub(crate) ns_: Vector<(i32, i32, i32)>,
    pub(crate) jn3d_: JPS3DNeib,

    /// free map. if it's `None`, a grid will be free as long as it is within the box defined by xdim × ydim × zdim.
    ///
    /// otherwise, when it's `Some`, the check for being contained in the free map is additionally needed to check whether a grid is free.
    pub fmap_: Option<HashSet<(i32, i32, i32)>>,
    /// occupied map
    pub omap_: HashSet<(i32, i32, i32)>,
    /// x-, y-, z- dimensions
    pub xdim: i32,
    pub ydim: i32,
    pub zdim: i32,

    /// a scalar in calculating the costs
    pub(crate) eps_: f32,

    /// goal
    pub goal: (i32, i32, i32),

    /// use_jps
    pub use_jps_: bool,
}
impl GraphSearch {
    /// create a `GraphSearch` instance, with maps given in terms of HashMap.
    ///
    /// `fmap_` : free map, containing all coordinates of free grids.
    ///
    /// `omap_` : occupied map, containing all coordinates of occupied grids.
    ///
    /// `xdim`  : upper boundary on x-axis.
    ///
    /// `ydim`  : upper boundary on y-axis.
    ///
    /// `zdim`  : upper boundary on z-axis.
    ///
    /// `eps_`  : a scalar in calculating the costs.
    ///
    pub fn new_v1(
        fmap_: Option<HashSet<(i32, i32, i32)>>,
        omap_: HashSet<(i32, i32, i32)>,
        xdim: i32,
        ydim: i32,
        zdim: i32,
        eps_: f32,
        // goal: (i32, i32, i32),
        // use_jps_: bool,
    ) -> Self {
        // Set 3D neighbors
        let mut ns: Vector<(i32, i32, i32)> = Vector::new();
        for x in -1..2 {
            for y in -1..2 {
                for z in -1..2 {
                    if x == 0 && y == 0 && z == 0 {
                        continue;
                    }
                    ns.push_back((x, y, z));
                }
            }
        }

        Self {
            pq_: PriorityQueue::default(),
            hm_: HashMap::default(),
            path_: Vec::default(),
            turnings: Vec::default(),
            ns_: ns,
            jn3d_: JPS3DNeib::default(),
            fmap_: fmap_,
            omap_: omap_,
            xdim: xdim,
            ydim: ydim,
            zdim: zdim,
            eps_: eps_,
            ..Default::default() // goal: goal,
                                 // use_jps_: use_jps_,
        }
    }

    ///  create a `GraphSearch` instance, with the map given in terms of binary-valued array.
    ///
    /// `map`   : the map that is reshaped into 1d array, `true` means occupied, `false` means free.
    ///
    /// `xdim`  : upper boundary on x-axis.
    ///
    /// `ydim`  : upper boundary on y-axis.
    ///
    /// `zdim`  : upper boundary on z-axis.
    ///
    /// `eps_`  : a scalar in calculating the costs.
    ///
    pub fn new_v2(
        map: &[bool],
        xdim: i32,
        ydim: i32,
        zdim: i32,
        eps_: f32,
        // goal: (i32, i32, i32),
        // use_jps_: bool,
    ) -> Self {
        let mut occ_set: HashSet<(i32, i32, i32)> = HashSet::new();
        let mut free_set: HashSet<(i32, i32, i32)> = HashSet::new();

        for (b, idx) in map.iter().enumerate() {
            let z = b as i32 / 25;
            let y = (b as i32 - 25 * z) / 5;
            let x = b as i32 - 25 * z - 5 * y;
            if *idx {
                // occupied coordinates
                occ_set.insert((x, y, z));
            } else {
                // free coordinates
                free_set.insert((x, y, z));
            }
        }
        Self::new_v1(Some(free_set), occ_set, xdim, ydim, zdim, eps_)
    }
}

impl GraphSearch {
    #[allow(unused)]
    pub(crate) fn recover_path(
        &self,
        node: &State,
        start_id: (i32, i32, i32),
    ) -> (Vec<State>, Vec<(i32, i32, i32)>) {
        let mut path: Vec<State> = Vec::new();
        let mut turnings: Vec<(i32, i32, i32)> = Vec::new();
        let mut node_ref: Option<&State> = Some(node);
        while let Some(nd) = node_ref {
            path.push(nd.clone());
            if nd.id == start_id {
                break;
            } else {
                if let Some(par_id) = nd.parent_id {
                    node_ref = self.hm_.get(&par_id);
                }
            }
        }
        path.reverse();
        let len = path.len();
        let mut pre_dir: (i32, i32, i32) = (0, 0, 0);
        for (k, (pre, cur)) in path.iter().tuple_windows().enumerate() {
            // push goal
            if k == 0 {
                turnings.push(pre.id);
                pre_dir = (
                    cur.id.0 - pre.id.0,
                    cur.id.1 - pre.id.1,
                    cur.id.2 - pre.id.2,
                );
            } else {
                // push turnings
                let dir = (
                    cur.id.0 - pre.id.0,
                    cur.id.1 - pre.id.1,
                    cur.id.2 - pre.id.2,
                );

                if dir != pre_dir && k < len - 2 {
                    turnings.push(pre.id);
                    pre_dir = dir;
                }
            }
            // push start
            if k == len - 2 {
                turnings.push(cur.id);
            }
        }
        (path, turnings)
    }

    #[allow(unused)]
    /// the main entry for planning.
    ///
    /// `start` : starting grid
    ///
    /// `goal`  : goal grid
    ///
    /// `usejps`: `true` for using jps, `false` for using `A*`
    ///
    /// `max_expand` : max number of the depth in searching.  
    ///
    pub fn plan_main(
        &mut self,
        start: (i32, i32, i32),
        goal: (i32, i32, i32),
        usejps: bool,
        max_expand: i32,
    ) -> bool {
        self.pq_.clear();
        self.path_.clear();
        // Set jps
        self.use_jps_ = usejps;
        // Set goal
        self.goal = goal;
        // Set start node
        let mut current_node = State::new(start, start.0, start.1, start.2, 0, 0, 0);
        current_node.g = OrderedFloat(0.0);
        current_node.h = OrderedFloat(self.get_heur(&start));

        return self.plan_body(&mut current_node, max_expand, start, goal);
    }

    pub(crate) fn plan_body(
        &mut self,
        mut curr_node: &mut State,
        max_expand: i32,
        start_id: (i32, i32, i32),
        goal_id: (i32, i32, i32),
    ) -> bool {
        // Insert start node
        self.pq_
            .push(curr_node.clone(), Reverse(curr_node.g + curr_node.h));
        curr_node.opened = true;
        self.hm_.insert(start_id, curr_node.clone());
        let mut expand_iteration = 0;
        let mut curr_node: State;
        loop {
            expand_iteration += 1;
            // get element with smallest cost
            curr_node = self.pq_.pop().unwrap().0;
            curr_node.closed = true; // Add to closed list
            if curr_node.id == goal_id {
                // println!("Goal Reached!!!!!!");
                break;
            }
            let mut succ_ids: Vector<(i32, i32, i32)> = Vector::new();
            let mut succ_costs: Vector<f32> = Vector::new();
            // Get successors
            if !self.use_jps_ {
                self.get_succ(&curr_node, &mut succ_ids, &mut succ_costs);
            } else {
                self.get_jps_succ(&curr_node, &mut succ_ids, &mut succ_costs);
            }
            for s in 0..succ_ids.len() {
                // see if we can improve the value of succstate
                if let Some(child_ptr) = self.hm_.get_mut(&succ_ids[s]) {
                    // println!("child_ptr = {:?}", child_ptr);
                    let tentative_gval = curr_node.g + succ_costs[s];
                    if tentative_gval < child_ptr.g {
                        (*child_ptr).parent_id = Some(curr_node.id); // Assign new parent
                        (*child_ptr).g = tentative_gval; // Update gval

                        // if currently in OPEN, update
                        if child_ptr.opened && !child_ptr.closed {
                            child_ptr.dx = child_ptr.x - curr_node.x;
                            child_ptr.dy = child_ptr.y - curr_node.y;
                            child_ptr.dz = child_ptr.z - curr_node.z;
                            if child_ptr.dx != 0 {
                                child_ptr.dx /= child_ptr.dx.abs();
                            }
                            if child_ptr.dy != 0 {
                                child_ptr.dy /= child_ptr.dy.abs();
                            }
                            if child_ptr.dz != 0 {
                                child_ptr.dz /= child_ptr.dz.abs();
                            }
                            self.pq_
                                .push_increase(*child_ptr, Reverse(child_ptr.g + child_ptr.h));
                        }
                        // if currently in CLOSED
                        else if child_ptr.opened && child_ptr.closed {
                            println!("ASTAR ERROR!");
                        }
                        // new node, add to heap
                        else {
                            child_ptr.opened = true;
                            self.pq_
                                .push(child_ptr.clone(), Reverse(child_ptr.g + child_ptr.h));
                        }
                    }
                }
            }
            // println!("pq_ = {:?}", self.pq_);
            if max_expand > 0 && expand_iteration >= max_expand {
                // println!("MaxExpandStep {} Reached!!!!!!", max_expand);
                return false;
            }
            if self.pq_.is_empty() {
                // println!("Priority queue is empty!!!!!!");
                return false;
            }
        }
        // println!("goal g: {}, h: {}!", curr_node.g, curr_node.h);
        // println!("Expand {} nodes!", expand_iteration);

        // let curr_node = self.hm_.get(&curr_node.id).unwrap();

        (self.path_, self.turnings) = self.recover_path(&curr_node, start_id);
        true
    }

    pub(crate) fn get_succ(
        &mut self,
        curr: &State,
        succ_ids: &mut Vector<(i32, i32, i32)>,
        succ_costs: &mut Vector<f32>,
    ) {
        for d in self.ns_.iter() {
            let new_pos = (curr.x + d.0, curr.y + d.1, curr.z + d.2);
            if !self.is_free(&new_pos) {
                continue;
            }
            if !self.hm_.contains_key(&new_pos) {
                let mut new_state =
                    State::new(new_pos, new_pos.0, new_pos.1, new_pos.2, d.0, d.1, d.2);
                new_state.h = OrderedFloat(self.get_heur(&new_pos));
                self.hm_.insert(new_pos, new_state);
            }
            succ_ids.push_back(new_pos);
            succ_costs.push_back(((d.0.pow(2) + d.1.pow(2) + d.2.pow(2)) as f32).sqrt());
        }
    }

    pub(crate) fn get_jps_succ(
        &mut self,
        curr: &State,
        succ_ids: &mut Vector<(i32, i32, i32)>,
        succ_costs: &mut Vector<f32>,
    ) {
        let norm1 = curr.dx.abs() + curr.dy.abs() + curr.dz.abs();
        let num_neib = JPS3DNeib::nsz[norm1 as usize][0] as usize;
        let num_fneib = JPS3DNeib::nsz[norm1 as usize][1] as usize;
        let id = ((curr.dx + 1) + 3 * (curr.dy + 1) + 9 * (curr.dz + 1)) as usize;

        for dev in 0..num_neib + num_fneib {
            let (mut new_x, mut new_y, mut new_z) = (0, 0, 0);
            let (dx, dy, dz);
            if dev < num_neib {
                dx = self.jn3d_.ns[id][0][dev];
                dy = self.jn3d_.ns[id][1][dev];
                dz = self.jn3d_.ns[id][2][dev];
                if !self.jump(
                    curr.x, curr.y, curr.z, dx, dy, dz, &mut new_x, &mut new_y, &mut new_z,
                ) {
                    continue;
                }
            } else {
                let nx = curr.x + self.jn3d_.f1[id][0][dev - num_neib];
                let ny = curr.y + self.jn3d_.f1[id][1][dev - num_neib];
                let nz = curr.z + self.jn3d_.f1[id][2][dev - num_neib];
                if self.is_occupied(&(nx, ny, nz)) {
                    dx = self.jn3d_.f2[id][0][dev - num_neib];
                    dy = self.jn3d_.f2[id][1][dev - num_neib];
                    dz = self.jn3d_.f2[id][2][dev - num_neib];
                    if !self.jump(
                        curr.x, curr.y, curr.z, dx, dy, dz, &mut new_x, &mut new_y, &mut new_z,
                    ) {
                        continue;
                    }
                } else {
                    continue;
                }
            }
            let new_pos = (new_x, new_y, new_z);
            if !self.hm_.contains_key(&new_pos) {
                let mut new_state =
                    State::new(new_pos, new_pos.0, new_pos.1, new_pos.2, dx, dy, dz);
                new_state.h = OrderedFloat(self.get_heur(&new_pos));
                self.hm_.insert(new_pos, new_state);
            }
            succ_ids.push_back(new_pos);
            succ_costs.push_back(
                (((new_x - curr.x).pow(2) + (new_y - curr.y).pow(2) + (new_z - curr.z).pow(2))
                    as f32)
                    .sqrt(),
            );
            // println!(
            //     "num_neib = {}, num_fneib = {}, succ_ids = {:?}",
            //     num_neib, num_fneib, succ_ids
            // );
        }
    }

    pub(crate) fn jump(
        &self,
        x: i32,
        y: i32,
        z: i32,
        dx: i32,
        dy: i32,
        dz: i32,
        new_x: &mut i32,
        new_y: &mut i32,
        new_z: &mut i32,
    ) -> bool {
        *new_x = x + dx;
        *new_y = y + dy;
        *new_z = z + dz;
        if !self.is_free(&(*new_x, *new_y, *new_z)) {
            return false;
        }
        if (*new_x, *new_y, *new_z) == self.goal {
            return true;
        }
        if self.has_forced(*new_x, *new_y, *new_z, dx, dy, dz) {
            return true;
        }
        let id = ((dx + 1) + 3 * (dy + 1) + 9 * (dz + 1)) as usize;
        let norm1 = dx.abs() + dy.abs() + dz.abs();
        let num_neib = JPS3DNeib::nsz[norm1 as usize][0];
        for k in 0..num_neib as usize - 1 {
            let (mut new_new_x, mut new_new_y, mut new_new_z) = (0, 0, 0);
            if self.jump(
                *new_x,
                *new_y,
                *new_z,
                self.jn3d_.ns[id][0][k],
                self.jn3d_.ns[id][1][k],
                self.jn3d_.ns[id][2][k],
                &mut new_new_x,
                &mut new_new_y,
                &mut new_new_z,
            ) {
                return true;
            }
        }
        return self.jump(*new_x, *new_y, *new_z, dx, dy, dz, new_x, new_y, new_z);
    }

    #[inline]
    pub(crate) fn has_forced(&self, x: i32, y: i32, z: i32, dx: i32, dy: i32, dz: i32) -> bool {
        let norm1 = dx.abs() + dy.abs() + dz.abs();
        let id = ((dx + 1) + 3 * (dy + 1) + 9 * (dz + 1)) as usize;
        match norm1 {
            // 1-d move, check 8 neighbors
            1 => {
                for fk in 0..8_usize {
                    let nx = x + self.jn3d_.f1[id][0][fk];
                    let ny = y + self.jn3d_.f1[id][1][fk];
                    let nz = z + self.jn3d_.f1[id][2][fk];
                    if self.is_occupied(&(nx, ny, nz)) {
                        return true;
                    }
                }
                return false;
            }
            // 2-d move, check 8 neighbors
            2 => {
                for fk in 0..8_usize {
                    let nx = x + self.jn3d_.f1[id][0][fk];
                    let ny = y + self.jn3d_.f1[id][1][fk];
                    let nz = z + self.jn3d_.f1[id][2][fk];
                    if self.is_occupied(&(nx, ny, nz)) {
                        return true;
                    }
                }
                return false;
            }
            // 3-d move, check 6 neighbors
            3 => {
                for fk in 0..6_usize {
                    let nx = x + self.jn3d_.f1[id][0][fk];
                    let ny = y + self.jn3d_.f1[id][1][fk];
                    let nz = z + self.jn3d_.f1[id][2][fk];
                    if self.is_occupied(&(nx, ny, nz)) {
                        return true;
                    }
                }
                return false;
            }
            _ => {
                return false;
            }
        }
    }

    #[inline]
    pub(crate) fn is_free(&self, pos: &(i32, i32, i32)) -> bool {
        let is_inside = || {
            pos.0 >= 0
                && pos.0 < self.xdim
                && pos.1 >= 0
                && pos.1 < self.ydim
                && pos.2 >= 0
                && pos.2 < self.zdim
        };
        match self.fmap_.as_ref() {
            Some(fmap) => fmap.contains(&pos) && is_inside(),
            None => is_inside() && !self.is_occupied(pos),
        }
    }

    #[inline]
    pub(crate) fn is_occupied(&self, pos: &(i32, i32, i32)) -> bool {
        pos.0 >= 0
            && pos.0 < self.xdim
            && pos.1 >= 0
            && pos.1 < self.ydim
            && pos.2 >= 0
            && pos.2 < self.zdim
            && self.omap_.contains(&pos)
    }

    #[inline]
    pub(crate) fn get_heur(&self, pos: &(i32, i32, i32)) -> f32 {
        self.eps_
            * (((pos.0 - self.goal.0).pow(2)
                + (pos.1 - self.goal.1).pow(2)
                + (pos.2 - self.goal.2).pow(2)) as f32)
                .sqrt()
    }
}
