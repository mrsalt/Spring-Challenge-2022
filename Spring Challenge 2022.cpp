#define _USE_MATH_DEFINES
#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <chrono>

using namespace std;

typedef chrono::high_resolution_clock Clock;

Clock::time_point start;

string elapsed() {
    return string("[") + to_string(chrono::duration_cast<chrono::milliseconds>(Clock::now() - start).count()) + "] ";
}

const int BASE_RADIUS = 5000;
const int RING_SIZE = 2400;
const int ATTACK_RADIUS = 800;
const int HERO_TRAVEL = 800;
const int HERO_TRAVEL_SQUARED = HERO_TRAVEL * HERO_TRAVEL;

const int RIGHT_EDGE = 17630;
const int BOTTOM_EDGE = 9000;

enum class EntityType {
    Monster = 0,
    Hero = 1,
    OpponentHero = 2
};

ostream& operator << (ostream& out, const EntityType c) {
    switch (c) {
    case EntityType::Monster: out << "Monster"; break;
    case EntityType::Hero: out << "Hero"; break;
    case EntityType::OpponentHero: out << "OpponentHero"; break;
    }
    return out;
}

enum class ThreatType {
    NotHeadingToBase = 0,
    MyBase = 1,
    OpponentBase = 2
};

ostream& operator << (ostream& out, const ThreatType c) {
    switch (c) {
    case ThreatType::NotHeadingToBase: out << "NotHeadingToBase"; break;
    case ThreatType::MyBase: out << "MyBase"; break;
    case ThreatType::OpponentBase: out << "OpponentBase"; break;
    }
    return out;
}

struct Delta
{
    int dx;
    int dy;
    friend istream& operator >> (istream& in, Delta& d) {
        in >> d.dx >> d.dy;
        return in;
    }

    friend ostream& operator << (ostream& out, const Delta& c) {
        out << "Delta{" << c.dx << ", " << c.dy << "}";
        return out;
    }

    int squared() const {
        return dx * dx + dy * dy;
    }

    int distance() const {
        return sqrt(squared());
    }
};

struct Point
{
    int x;
    int y;

    Point& operator =(const Point& other) {
        x = other.x;
        y = other.y;
        return *this;
    }

    friend istream& operator >> (istream& in, Point& c) {
        in >> c.x >> c.y;
        return in;
    }

    friend ostream& operator << (ostream& out, const Point& c) {
        out << "Point{" << c.x << ", " << c.y << "}";
        return out;
    }

    Point operator+(const Delta& d) const {
        return { x + d.dx, y + d.dy };
    }

    Point& operator+=(const Delta& d) {
        x += d.dx;
        y += d.dy;
        return *this;
    }

    Delta operator-(const Point& p) const {
        return { x - p.x, y - p.y };
    }

    bool operator<(const Point& other) const {
        if (y == other.y) {
            return x < other.x;
        }
        return y < other.y;
    }
};

vector<Point> default_hero_location;

struct Polar
{
    int dist;
    double theta;

    Polar() : dist(0), theta(0.0) {
    }

    Polar(const Point& center, const Point& p) {
        init(center, p);
    }

    Polar(int distance, double theta) {
        this->dist = distance;
        this->theta = theta;
    }

    void init(const Point& center, const Point& p) {
        int dx = p.x - center.x;
        int dy = p.y - center.y;
        dist = sqrt(dx * dx + dy * dy);

        if (dx == 0) {
            if (dy < 0)
                theta = M_PI * 3.0 / 2.0;
            else
                theta = M_PI / 2.0;
        }
        else if (dy == 0) {
            if (dx < 0)
                theta = M_PI;
            else
                theta = 0;
        }
        else {
            theta = atan((double)dy / (double)dx);
            if (dx < 0) {
                theta += M_PI;
            }
            else if (dy < 0) {
                theta += 2.0 * M_PI;
            }
        }
    }

    Delta toDelta() const {
        return { (int)(dist * cos(theta)), (int)(dist * sin(theta)) };
    }

    Point toPoint(const Point& center) const {
        return center + toDelta();
    }

    friend ostream& operator << (ostream& out, const Polar& c) {
        out << "Polar{dist: " << c.dist << ", angle: " << (int)(c.theta * 360.0 / (2 * M_PI)) << "}";
        return out;
    }
};

struct Entity
{
    int id; // Unique identifier
    EntityType type; // 0=monster, 1=your hero, 2=opponent hero
    Point position; // Position of this entity
    int shield_life; // Ignore for this league; Count down until shield spell fades
    int is_controlled; // Ignore for this league; Equals 1 when this entity is under a control spell
    int health; // Remaining health of this monster
    int adjusted_health;
    Delta velocity; // Trajectory of this monster
    int near_base; // 0=monster with no target yet, 1=monster targeting a base
    ThreatType threat_for; // Given this monster's trajectory, is it a threat to 1=your base, 2=your opponent's base, 0=neither

    Point end_position;
    Polar end_polar;

    bool targettingMyBase() const {
        return type == EntityType::Monster && near_base == 1;
    }

    bool willTargetMyBase() const {
        return type == EntityType::Monster && near_base == 0 && threat_for == ThreatType::MyBase;
    }

    friend istream& operator >> (istream& in, Entity& c) {
        int entityType;
        in >> c.id >> entityType;
        c.type = (EntityType)entityType;
        in >> c.position;
        in >> c.shield_life >> c.is_controlled >> c.health;
        c.adjusted_health = c.health;
        int threatType;
        in >> c.velocity >> c.near_base >> threatType;
        c.threat_for = (ThreatType)threatType;
        in.ignore();
        c.end_position = c.position + c.velocity;
        return in;
    }

    friend ostream& operator << (ostream& out, const Entity& c) {
        const char* spacer = "    ";
        out << "Entity{" << "\n";
        out << spacer << "id: " << c.id << "\n";
        out << spacer << "type: " << c.type << "\n";
        out << spacer << "position: " << c.position << "\n";
        out << spacer << "shield_life: " << c.shield_life << "\n";
        out << spacer << "health: " << c.health << "\n";
        out << spacer << "is_controlled: " << c.is_controlled << "\n";
        out << spacer << "velocity: " << c.velocity << "\n";
        out << spacer << "threat_for: " << c.threat_for << "\n";
        out << spacer << "end_position: " << c.end_position << "\n";
        out << spacer << "end_polar: " << c.end_polar << "\n";
        out << "}";
        return out;
    }
};

template <typename T> struct Ring;

template <typename T>
struct Segment {
    T data;
    double startAngle;
    double endAngle;
    const Ring<T>* ring;
    Point center;

    void initialize(const Point& base, double startAngle, double endAngle, const Ring<T>* ring) {
        this->startAngle = startAngle;
        this->endAngle = endAngle;
        this->ring = ring;
        this->center = toPoint(base);
    }

private:
    Point toPoint(const Point& center) {
        double theta = (startAngle + endAngle) / 2;
        int dist = (ring->innerDistance + ring->outerDistance) / 2;
        return Polar(dist, theta).toPoint(center);
    }
};

template <typename T>
struct Ring {
    vector<Segment<T>> segmentList;
    int innerDistance;
    int outerDistance;
    int monsterCount;
};

template <typename T>
struct RingList {
    Point center;
    double startAngle;
    double endAngle;
    double segmentArc;
    vector<Ring<T>> rings;

    RingList(const Point& center, int initialDistance, int ringDistance, int ringCount, int segments, double startAngle = 0.0, double endAngle = M_PI * 2.0) {
        this->center = center;
        this->segmentArc = (endAngle - startAngle) / (double)segments;
        //cerr << elapsed() << "RingList: resizing rings.  Rings: " << ringCount << ", segments: " << segments << endl;
        rings.resize(ringCount);
        int currentDistance = initialDistance;
        for (int i = 0; i < ringCount; i++) {
            auto& ring = rings[i];
            if (i == 0)
                ring.innerDistance = 0;
            else
                ring.innerDistance = rings[i - 1].outerDistance;
            ring.outerDistance = currentDistance;
            currentDistance += ringDistance;
        }
        //cerr << elapsed() << "RingList: initializing segments" << endl;
        for (Ring<T>& ring : rings) {
            ring.segmentList.resize(segments);
            for (int i = 0; i < segments; i++) {
                auto& segment = ring.segmentList[i];
                segment.initialize(center, i * segmentArc, (i + 1) * segmentArc, &ring);
            }
        }
        //cerr << elapsed() << "RingList: initialization complete" << endl;
    }
};

Point findInterdictionPoint(const Entity& hero, const Entity& threat) {
    Point end_position = threat.position;
    for (int turn = 0; turn < 50; turn++) {
        end_position += threat.velocity;
        if ((end_position - hero.position).distance() < HERO_TRAVEL * (turn + 1))
            return end_position;
    }
    cerr << hero << " is unable to interdict " << threat << endl;
    throw exception();
}

struct EntityThreatList : RingList<vector<Entity>> {

    EntityThreatList(const Point& center, int initialDistance, int ringDistance, int ringsToTrack, int segments, double startAngle, double endAngle)
        : RingList<vector<Entity>>(center, initialDistance, ringDistance, ringsToTrack, segments, startAngle, endAngle)
    {
    }

    vector<Entity> threats;
    vector<Entity> heroes;

    void placeEntity(const Entity& e) {
        if (e.type == EntityType::Hero) {
            heroes.push_back(e);
        }
        else if (e.targettingMyBase() || e.willTargetMyBase()) {
            if (e.end_polar.dist >= rings.back().outerDistance) {
                cerr << elapsed() << "Monster is too far (" << e.end_polar.dist << ") to reach any ring (outer distance: " << rings.back().outerDistance << ")" << endl;
                return;
            }
            for (int i = 0; i < rings.size(); i++) {
                if (e.end_polar.dist < rings[i].outerDistance) {
                    auto& ring = rings[i];
                    cerr << elapsed() << "Monster " << e.id << " (end distance: " << e.end_polar.dist << ") in ring " << i << " (distance: " << ring.innerDistance << " - " << ring.outerDistance << ")" << endl;
                    int segmentNum = e.end_polar.theta / segmentArc;

                    auto& segment = ring.segmentList[segmentNum];
                    ring.monsterCount++;
                    segment.data.push_back(e);
                    threats.push_back(e);
                    return;
                }
            }
        }
    }

    vector<Point> placeDefenders() {
        cerr << elapsed() << "Placing defenders" << endl;

        if (threats.empty()) {
            cerr << elapsed() << "No threats.  Ordering all heroes to default locations." << endl;
            return default_hero_location;
        }

        if (threats.size() == 1) {
            cerr << elapsed() << "Only 1 threat.  Ordering all heroes to interdict." << endl;
            vector<Point> hero_placements(heroes.size());
            for (int h = 0; h < heroes.size(); h++) {
                hero_placements[h] = findInterdictionPoint(heroes[h], threats.front());
            }
            return hero_placements;
        }

        vector<set<Point>> pointsReachableByHeroes(heroes.size());
        map<Point, int> monstersAttackedAtPoint;

        for (int i = 0; i < rings.size(); i++) {
            Ring<vector<Entity>>& ring = rings[i];
            if (ring.monsterCount > 0) {
                cerr << elapsed() << ring.monsterCount << " Monster(s) found in ring " << i << endl;
                map<int, vector<int>> segmentsWithMonsters;
                for (int j = 0; j < ring.segmentList.size(); j++) {
                    Segment<vector<Entity>>& segment = ring.segmentList[j];
                    size_t count = segment.data.size();
                    if (count > 0) {
                        segmentsWithMonsters[count].push_back(j);
                    }
                }

                cerr << elapsed() << "Examining " << segmentsWithMonsters.size() << " group(s) of segments." << endl;

                // Let's examine the segments with monsters; break them apart
                // into smaller segments and discover which locations reach the most monsters.
                for (auto it = segmentsWithMonsters.rbegin(); it != segmentsWithMonsters.rend(); it++) {
                    const vector<int>& segmentNumbers = it->second;
                    cerr << elapsed() << "In ring " << i << ", " << segmentNumbers.size() << " segment(s) contain(s) " << it->first << " monster(s)." << endl;
                    for (int segmentNum : segmentNumbers) {
                        Segment<vector<Entity>>& segment = ring.segmentList[segmentNum];
                        map<int, vector<Point>> monsterMap = findBestLocationsToPlaceDefenders(segment, ring);
                        if (monsterMap.empty())
                            continue;
                        //cerr << elapsed() << "Monster map has " << monsterMap.size() << " elements" << endl;
                        auto pointIt = monsterMap.rbegin();
                        const vector<Point>& points = pointIt->second;
                        cerr << elapsed() << points.size() << " points(s) in segment " << segmentNum << " would attack " << pointIt->first << " monster(s):" << endl;
                        for (const Point& p : points) {
                            monstersAttackedAtPoint[p] = pointIt->first;
                            cerr << elapsed() << p << ": ";
                            for (int h = 0; h < heroes.size(); h++) {
                                const Entity& hero = heroes[h];
                                Delta dist = hero.position - p;
                                if (dist.squared() < HERO_TRAVEL_SQUARED) {
                                    pointsReachableByHeroes[h].insert(p);
                                    cerr << "[hero " << h << "]";
                                }
                            }
                            cerr << endl;
                        }
                    }
                    cerr << elapsed() << "Done examining group of segments " << endl;
                }
                cerr << elapsed() << "Done examining ring " << i << endl;
            }
        }

        cerr << elapsed() << "Findings points uniquely reachable " << endl;

        vector<vector<Point>> pointsUniquelyReachableByHeroes(heroes.size());
        for (int h = 0; h < heroes.size(); h++) {
            cerr << elapsed() << "Hero " << h << " can reach " << pointsReachableByHeroes[h].size() << " points." << endl;
            set<Point> pointsReachableByOtherHeroes;
            for (int i = 0; i < heroes.size(); i++) {
                if (i == h)
                    continue;
                const set<Point>& p = pointsReachableByHeroes[i];
                pointsReachableByOtherHeroes.insert(p.begin(), p.end());
            }
            pointsUniquelyReachableByHeroes[h].resize(pointsReachableByHeroes[h].size());
            vector<Point>::iterator end = (pointsReachableByHeroes[h].begin(), pointsReachableByHeroes[h].end(), pointsReachableByOtherHeroes.begin(), pointsReachableByOtherHeroes.end(), pointsUniquelyReachableByHeroes[h].begin());
            size_t unique = (end - pointsUniquelyReachableByHeroes[h].begin());
            pointsUniquelyReachableByHeroes[h].resize(unique);
            cerr << elapsed() << "Hero " << h << " (" << heroes[h].position << ") can reach " << pointsUniquelyReachableByHeroes[h].size() << " unique points." << endl;
        }

        vector<Point> hero_placements(heroes.size());
        for (int h = 0; h < heroes.size(); h++) {
            Point placement;
            if (!pointsUniquelyReachableByHeroes[h].empty()) {
                placement = findBestPointToAttack(monstersAttackedAtPoint, pointsUniquelyReachableByHeroes[h]);
                cerr << elapsed() << "Assigning hero " << h << " a location uniquely reachable (" << placement << ")" << "\n";
            }
            else if (!pointsReachableByHeroes[h].empty()) {
                placement = findBestPointToAttack(monstersAttackedAtPoint, pointsReachableByHeroes[h]);
                cerr << elapsed() << "Assigning hero " << h << " a reachable location (" << placement << ")" << "\n";
            }
            else {
                for (int r = 0; r < rings.size(); r++) {
                    Ring<vector<Entity>>& ring = rings[r];
                    int min_distance;
                    const Entity* closest = findClosestMonsterInSegment(heroes[h], ring.segmentList[h].data, min_distance);
                    if (closest == nullptr) {
                        for (int s = 0; s < ring.segmentList.size(); s++) {
                            if (s == h)
                                continue; // we examined this segment first
                            closest = findClosestMonsterInSegment(heroes[h], ring.segmentList[s].data, min_distance, closest);
                        }
                    }
                    if (closest != nullptr) {
                        placement = closest->end_position;
                        cerr << elapsed() << "Assigning hero " << h << " to the closest monster in ring " << r << " (" << placement << ")" << "\n";
                        break;
                    }
                    else if (r == rings.size() - 1) {
                        placement = default_hero_location[h];
                        cerr << elapsed() << "Assigning hero " << h << " its default position (" << placement << ")" << "\n";
                    }
                }
            }
            hero_placements[h] = placement;
        }
        return hero_placements;
    }

    const Entity* findClosestMonsterInSegment(const Entity & hero, const vector<Entity> & monsters, int & min_distance, const Entity* closest = nullptr)
    {
        for (auto& monster : monsters) {
            int distance = (hero.position - monster.end_position).squared();
            if (closest == nullptr || distance < min_distance) {
                closest = &monster;
                min_distance = distance;
            }
        }
        return closest;
    }

    template<typename ContainerType>
    Point findBestPointToAttack(map<Point, int>& monstersAttackedAtPoint, ContainerType pointList) {
        int max_monsters = 0;
        Point placement;
        for (Point p : pointList) {
            if (monstersAttackedAtPoint[p] > max_monsters) {
                max_monsters = monstersAttackedAtPoint[p];
                placement = p;
            }
        }
        return placement;
    }

    map<int, vector<Point>> findBestLocationsToPlaceDefenders(Segment<vector<Entity>>& segment, Ring<vector<Entity>>& ring) {
        const int attack_squared = ATTACK_RADIUS * ATTACK_RADIUS;

        int ringsToTrack = 6;
        int ringDistance = (ring.outerDistance - ring.innerDistance) / ringsToTrack;
        RingList<int> reachableMonsters(center, ring.innerDistance, ringDistance, ringsToTrack, 6, segment.startAngle, segment.endAngle);

        cerr << elapsed() << "Finding points that reach the most monsters" << endl;

        for (Ring<int>& ring : reachableMonsters.rings) {
            //cerr << elapsed() << "  Examining ring" << endl;
            for (Segment<int>& s : ring.segmentList) {
                //cerr << elapsed() << "    Examining segment" << endl;
                for (Entity e : threats) {
                    Delta distance = e.end_position - s.center;
                    if (distance.squared() < attack_squared) {
                        // if a hero were at this location, it would reach this threat.
                        s.data++;
                    }
                }
            }
        }

        cerr << elapsed() << "Initializing map by monster count to points" << endl;
        map<int, vector<Point>> monsterMap;
        for (Ring<int>& ring : reachableMonsters.rings) {
            for (Segment<int>& s : ring.segmentList) {
                if (s.data > 0) {
                    monsterMap[s.data].push_back(s.center);
                }
            }
        }
        cerr << elapsed() << "Done initializing map by monster count to points" << endl;
        return monsterMap;
    }
};

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

 /*
 void debug() {

     for (int angle = 0; angle < 360; angle += 10) {
         double theta = angle * 2.0 * M_PI / 360.0;
         cerr << angle << ": x: " << (int)(cos(theta) * 100) << ", y: " << (int)(sin(theta) * 100) << endl;
     }

     Point b{ 0, 0 };
     Point p1{ 10, 0 };
     Point p15{ 10, 10 };
     Point p2{ 0, 10 };
     Point p25{ -10, 10 };
     Point p3{ -10, 0 };
     Point p35{ -10, -10 };
     Point p4{ 0, -10 };
     Point p45{ 10, -10 };
     cerr << Polar(b, p1) << endl;
     cerr << Polar(b, p15) << endl;
     cerr << Polar(b, p2) << endl;
     cerr << Polar(b, p25) << endl;
     cerr << Polar(b, p3) << endl;
     cerr << Polar(b, p35) << endl;
     cerr << Polar(b, p4) << endl;
     cerr << Polar(b, p45) << endl;
 }*/

void find_default_hero_placements(Point& base, int heroes_per_player, double& start_angle, double& end_angle)
{
    if (base.x == 0 && base.y == 0) {
        start_angle = 0;
        end_angle = M_PI_2;
    }
    else if (base.x == RIGHT_EDGE && base.y == 0) {
        start_angle = M_PI_2;
        end_angle = M_PI;
    }
    else if (base.x == RIGHT_EDGE && base.y == BOTTOM_EDGE) {
        start_angle = M_PI;
        end_angle = M_PI * 3.0 / 2.0;
    }
    else if (base.x == 0 && base.y == BOTTOM_EDGE) {
        start_angle = M_PI * 3.0 / 2.0;
        end_angle = M_PI * 2.0;
    }
    else {
        start_angle = 0.0;
        end_angle = M_PI * 2.0;
    }

    double defender_arc = (end_angle - start_angle) / heroes_per_player;
    double current = defender_arc / 2.0;
    for (int i = 0; i < heroes_per_player; i++) {
        Point p = Polar(BASE_RADIUS + HERO_TRAVEL, current).toPoint(base);
        cerr << "Default hero position[" << i << "]: " << p << endl;
        default_hero_location.push_back(p);
        current += defender_arc;
    }
}

int main()
{
    start = Clock::now();
    Point base; // The corner of the map representing your base
    cin >> base; cin.ignore();
    cerr << elapsed() << "Base: " << base << endl;

    int heroes_per_player; // Always 3
    cin >> heroes_per_player; cin.ignore();
    cerr << elapsed() << "Heroes: " << heroes_per_player << endl;

    double start_angle, end_angle;
    find_default_hero_placements(base, heroes_per_player, start_angle, end_angle);

    // game loop
    while (1) {
        for (int i = 0; i < 2; i++) {
            int health; // Each player's base health
            int mana; // Ignore in the first league; Spend ten mana to cast a spell
            cin >> health >> mana; cin.ignore();
            start = Clock::now();
            cerr << elapsed() << "Base Health: " << health << ", Mana: " << mana << endl;
        }
        int entity_count; // Amount of heros and monsters you can see
        cin >> entity_count; cin.ignore();

        cerr << elapsed() << "Entity Count: " << entity_count << endl;
        vector<Entity> entities(entity_count);

        cerr << elapsed() << "Initializing threat list" << endl;
        EntityThreatList threatList(base, BASE_RADIUS, RING_SIZE, 3, heroes_per_player, start_angle, end_angle);

        cerr << elapsed() << "Reading entities" << endl;
        for (int i = 0; i < entity_count; i++) {
            Entity& entity = entities[i];
            cin >> entity;
            entity.end_polar = Polar(base, entity.end_position);
            if (entity.targettingMyBase() || entity.willTargetMyBase())
                cerr << elapsed() << "Entity[" << i << "]: " << entity << endl;
            threatList.placeEntity(entity);
        }
        cerr << elapsed() << "Read entities" << endl;

        vector<Point> placements = threatList.placeDefenders();

        cerr << elapsed() << "Defenders placed" << endl;

        for (int i = 0; i < heroes_per_player; i++) {

            // Write an action using cout. DON'T FORGET THE "<< endl"
            cout << "MOVE " << placements[i].x << " " << placements[i].y << endl;

            // In the first league: MOVE <x> <y> | WAIT; In later leagues: | SPELL <spellParams>;
        }
    }
}