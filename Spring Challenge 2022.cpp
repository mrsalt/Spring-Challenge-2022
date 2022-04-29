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
#include <memory>

using namespace std;

typedef chrono::high_resolution_clock Clock;

Clock::time_point start;

string elapsed() {
    return string("[") + to_string(chrono::duration_cast<chrono::milliseconds>(Clock::now() - start).count()) + "] ";
}

const int BASE_RADIUS = 5000;
const int RING_SIZE = 2400;
const int ATTACK_RADIUS = 800;
const int ATTACK_SQUARED = ATTACK_RADIUS * ATTACK_RADIUS;
const int HERO_TRAVEL = 800;
const int HERO_TRAVEL_SQUARED = HERO_TRAVEL * HERO_TRAVEL;
const int HERO_DAMAGE = 2; // heroes do 2 points of damage to nearby enemies

const int RIGHT_EDGE = 17630;
const int BOTTOM_EDGE = 9000;

int radiansToDegrees(double radians) {
    return (radians * 360.0 / (2 * M_PI)) + 0.5;
}

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

    Delta operator*(int scalar) const {
        return { dx * scalar, dy * scalar };
    }

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

    bool operator!=(const Point& other) const {
        return x != other.x || y != other.y;
    }

    string toString() const {
        return to_string(x) + " " + to_string(y);
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
        out << "Polar{dist: " << c.dist << ", angle: " << radiansToDegrees(c.theta) << "}";
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
    vector<int> adjusted_health;
    Delta velocity; // Trajectory of this monster
    int near_base; // 0=monster with no target yet, 1=monster targeting a base
    ThreatType threat_for; // Given this monster's trajectory, is it a threat to 1=your base, 2=your opponent's base, 0=neither

    Point end_position;
    Polar end_polar;

    vector<int> damagedBy;

    bool targettingMyBase() const {
        return type == EntityType::Monster && near_base == 1;
    }

    bool willTargetMyBase() const {
        return type == EntityType::Monster && near_base == 0 && threat_for == ThreatType::MyBase;
    }

    void damage(const Entity& hero, int turnsToReach) {
        // Each hero can damage a given entity only once.
        if (find(damagedBy.begin(), damagedBy.end(), hero.id) != damagedBy.end())
            return;
        damagedBy.push_back(hero.id);

        // When we start attacking a monster, we project that we will continue to attack it each turn until it is dead.
        if (adjusted_health.empty()) {
            int projectedHealth = health;
            int turnCount = 0;
            while (projectedHealth > 0) {
                if (turnCount >= turnsToReach)
                    projectedHealth -= HERO_DAMAGE;
                adjusted_health.push_back(projectedHealth);
                turnCount++;
            }
        }
        else {
            int cumulative_damage = 0;
            for (int turnCount = 0; turnCount < adjusted_health.size(); turnCount++) {
                if (turnCount >= turnsToReach) {
                    cumulative_damage += HERO_DAMAGE;
                    adjusted_health[turnCount] -= cumulative_damage;
                }
                if (adjusted_health[turnCount] < 0) {
                    adjusted_health.resize(turnCount + 1);
                    break;
                }
            }
        }
        cerr << elapsed() << "Hero " << hero.id << " will cause damage to monster " << id << " (" << this << ").  Current health: " << health << endl;
        cerr << elapsed() << "  Projected health: " << endl;
        for (int i = 0; i < adjusted_health.size(); i++) {
            cerr << elapsed() << "    turn[" << i << "] = " << adjusted_health[i] << endl;
        }
    }

    bool isAliveInNTurns(int turns) const {
        if (adjusted_health.empty())
            return true;
        return turns < adjusted_health.size() - 1;
    }

    int healthInNTurns(int turns) const {
        if (adjusted_health.empty())
            return health;
        if (turns > adjusted_health.size())
            return -999;
        return adjusted_health[turns];
    }

    Point futurePosition(int turns) const {
        return position + (velocity * turns);
    }

    friend istream& operator >> (istream& in, Entity& c) {
        int entityType;
        in >> c.id >> entityType;
        c.type = (EntityType)entityType;
        in >> c.position;
        in >> c.shield_life >> c.is_controlled >> c.health;
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

class Action
{
public:
    virtual string toString() = 0;
};

class WaitAction : public Action
{
public:
    string toString() { return "WAIT"; }
};

class MoveAction : public Action
{
public:
    const Point location;
    MoveAction(const Point& p) : location(p) {}
    string toString() { return "MOVE " + location.toString(); }
};

class WindSpellAction : public Action
{
public:
    const Point target;
    WindSpellAction(const Point& target) : target(target) {}
    string toString() { return "SPELL WIND " + target.toString(); }
};

class ShieldSpellAction : public Action
{
public:
    const int targetId;
    ShieldSpellAction(const Entity& target) : targetId(target.id) {}
    string toString() { return "SPELL SHIELD " + to_string(targetId); }
};


class ControlSpellAction : public Action
{
public:
    const int targetId;
    const Point location;
    ControlSpellAction(const Entity& target, const Point& location)
        : targetId(target.id)
        , location(location)
    {}
    string toString() { return "SPELL CONTROL " + to_string(targetId) + " " + location.toString(); }
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
    const Point center;
    const double startAngle;
    const double endAngle;
    const double segmentArc;
    vector<Ring<T>> rings;

    RingList(const Point& center, int initialDistance, int ringDistance, int ringCount, int segments, double startAngle = 0.0, double endAngle = M_PI * 2.0)
        : center(center)
        , startAngle(startAngle)
        , endAngle(endAngle)
        , segmentArc((endAngle - startAngle) / (double)segments)
    {
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
    Point position = threat.position;
    for (int turn = 0; turn < 50; turn++) {
        if ((position - hero.position).distance() < HERO_TRAVEL * (turn + 1))
            return position;
        position += threat.velocity;
    }
    cerr << hero << " is unable to interdict " << threat << endl;
    throw exception();
}

struct EntityThreatList : RingList<vector<Entity*>> {

    EntityThreatList(const Point& center, int initialDistance, int ringDistance, int ringsToTrack, int segments, double startAngle, double endAngle)
        : RingList<vector<Entity*>>(center, initialDistance, ringDistance, ringsToTrack, segments, startAngle, endAngle)
    {
        cerr << elapsed() << "Initializing threat list.  Start angle: " << radiansToDegrees(startAngle) << ", End angle: " << radiansToDegrees(endAngle) << endl;
    }

    vector<Entity*> threats;
    vector<Entity*> heroes;

    void placeEntity(Entity* e) {
        if (e->type == EntityType::Hero) {
            heroes.push_back(e);
        }
        else if (e->targettingMyBase() || e->willTargetMyBase()) {
            if (e->end_polar.dist >= rings.back().outerDistance) {
                cerr << elapsed() << "Monster is too far (" << e->end_polar.dist << ") to reach any ring (outer distance: " << rings.back().outerDistance << ")" << endl;
                return;
            }
            for (int i = 0; i < rings.size(); i++) {
                if (e->end_polar.dist < rings[i].outerDistance) {
                    auto& ring = rings[i];
                    cerr << elapsed() << "Monster " << e->id << " (end distance: " << e->end_polar.dist << ") in ring " << i << " (distance: " << ring.innerDistance << " - " << ring.outerDistance << ")" << endl;
                    int segmentNum = (e->end_polar.theta - startAngle) / segmentArc;
                    //cerr << elapsed() << "  segment #: " << segmentNum << ", startAngle: " << radiansToDegrees(startAngle) << ", e->end_polar: " << e->end_polar << ", segment arc: " << radiansToDegrees(segmentArc) << endl;
                    auto& segment = ring.segmentList[segmentNum];
                    ring.monsterCount++;
                    segment.data.push_back(e);
                    threats.push_back(e);
                    return;
                }
            }
        }
    }

    int countTurnsToInterdict(const Entity& threat, const Entity& hero) {
        Point position = threat.position;
        for (int turn = 0; turn < 50; turn++) {
            if ((position - hero.position).distance() < HERO_TRAVEL * (turn + 1))
                return turn;
            position += threat.velocity;
        }
        cerr << hero << " is unable to interdict " << threat << endl;
        throw exception();
    }

    pair<bool, vector<int>> canThreatBeEliminated(const Entity& threat) {
        Point position = threat.position;
        int health = threat.health;
        vector<int> turnsToInterdict;
        for (int h = 0; h < heroes.size(); h++)
            turnsToInterdict.push_back(countTurnsToInterdict(threat, *heroes[h]));
        vector<int> heroes_who_can_harm_threat;

        for (int turn = 0; turn < 50; turn++) {
            for (int h = 0; h < heroes.size(); h++)
                if (turnsToInterdict[h] <= turn) {
                    if (turnsToInterdict[h] == turn)
                        heroes_who_can_harm_threat.push_back(h);
                    health -= HERO_DAMAGE;
                }
            if (health <= 0)
                return make_pair(true, heroes_who_can_harm_threat);
            position += threat.velocity;
            if ((position - center).distance() <= 300)
                return make_pair(false, heroes_who_can_harm_threat);
        }
        cerr << "Unable to determine if threat can be eliminated " << threat << endl;
        throw exception();
    }

    unique_ptr<Action> assignMoveAction(const Entity& hero, const Point& p) {
        Delta d = p - hero.position;
        int distance = d.distance();

        Polar v(hero.position, p);
        v.dist = HERO_TRAVEL;
        Delta vector = v.toDelta();

        // How many turns until it can damage this location?
        int turns_to_damage = distance / HERO_TRAVEL;

        if (hero.position != p) {
            cerr << elapsed() << "hero moving from " << hero.position << " to " << p << endl;
            cerr << elapsed() << "  moving with velocity " << vector << " (distance = " << vector.distance() << ")" << endl;
            cerr << elapsed() << "turns_to_damage(" << turns_to_damage << ") = distance(" << distance << ") / HERO_TRAVEL(" << HERO_TRAVEL << ")" << endl;
        }


        for (Entity* threat : threats) {
            Point actualLocation = hero.position;

            for (int t = 0; t < turns_to_damage + 1; t++) {
                if (t == turns_to_damage)
                    actualLocation = p;
                else {
                    // hero will move towards this location, but will not reach it this turn.
                    // where will it reach this turn?
                    actualLocation += vector;
                }

                Point fp = threat->futurePosition(t);
                int distanceToThreat = (fp - actualLocation).distance();
                cerr << elapsed() << "threat future position[" << t << "] = " << fp << ", hero location = " << actualLocation << ", distanceToThreat = " << distanceToThreat << endl;
                if (distanceToThreat <= ATTACK_RADIUS) {
                    threat->damage(hero, t);
                    break;
                }
            }
        }

        if (hero.position != p) {
            return make_unique<MoveAction>(p);
        }
        else {
            return make_unique<WaitAction>();
        }
    }

    vector<unique_ptr<Action>> determineActions() {
        cerr << elapsed() << "Determining actions to take" << endl;

        vector<unique_ptr<Action>> actions(heroes.size());

        if (threats.empty()) {
            cerr << elapsed() << "No threats.  Ordering all heroes to default locations." << endl;
            for (int h = 0; h < heroes.size(); h++) {
                actions[h] = assignMoveAction(*heroes[h], default_hero_location[h]);
            }
            return actions;
        }

        if (threats.size() == 1) {
            const Entity& threat = *threats.front();
            const auto& analysis = canThreatBeEliminated(threat);
            if (analysis.first) {
                cerr << elapsed() << "Only 1 threat.  Ordering required heroes (" << analysis.second.size() << ") to interdict." << endl;
                for (int h : analysis.second) {
                    const Entity& hero = *heroes[h];
                    const Point& p = findInterdictionPoint(hero, threat);
                    cerr << elapsed() << "Assigning hero " << hero.id << " to attack threat at " << p << endl;
                    actions[h] = assignMoveAction(hero, p);
                }
                if (analysis.second.size() >= heroes.size())
                    return actions;
            }
            else {
                cerr << "Determined threat cannot be eliminated " << threat << endl;
            }
        }

        vector<set<Point>> pointsReachableByHeroes(heroes.size());
        map<Point, int> monstersAttackedAtPoint;

        for (int i = 0; i < rings.size(); i++) {
            Ring<vector<Entity*>>& ring = rings[i];
            if (ring.monsterCount > 0) {
                cerr << elapsed() << ring.monsterCount << " Monster(s) found in ring " << i << endl;
                map<int, vector<int>> segmentsWithMonsters;
                for (int j = 0; j < ring.segmentList.size(); j++) {
                    Segment<vector<Entity*>>& segment = ring.segmentList[j];
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
                        Segment<vector<Entity*>>& segment = ring.segmentList[segmentNum];
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
                                if (actions[h])
                                    continue;
                                const Entity& hero = *heroes[h];
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
            if (actions[h])
                continue;
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
            cerr << elapsed() << "Hero " << h << " (" << heroes[h]->position << ") can reach " << pointsUniquelyReachableByHeroes[h].size() << " unique points." << endl;
        }

        for (int h = 0; h < heroes.size(); h++) {
            if (actions[h]) {
                cerr << elapsed() << "Hero " << h << " is already assigned an action (" << actions[h]->toString() << ")" << endl;
                continue;
            }
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
                    Ring<vector<Entity*>>& ring = rings[r];
                    int min_distance;
                    const Entity* closest = findClosestMonsterInSegment(*heroes[h], ring.segmentList[h].data, min_distance);
                    if (closest == nullptr) {
                        for (int s = 0; s < ring.segmentList.size(); s++) {
                            if (s == h)
                                continue; // we examined this segment first
                            closest = findClosestMonsterInSegment(*heroes[h], ring.segmentList[s].data, min_distance, closest);
                        }
                    }
                    if (closest != nullptr) {
                        placement = closest->position;
                        cerr << elapsed() << "Assigning hero " << h << " to the closest monster in ring " << r << " (" << placement << ")" << "\n";
                        break;
                    }
                    else if (r == rings.size() - 1) {
                        placement = default_hero_location[h];
                        cerr << elapsed() << "Assigning hero " << h << " its default position (" << placement << ")" << "\n";
                    }
                }
            }
            actions[h] = assignMoveAction(*heroes[h], placement);
        }
        return actions;
    }

    const Entity* findClosestMonsterInSegment(const Entity& hero, const vector<Entity*>& monsters, int& min_distance, const Entity* closest = nullptr)
    {
        for (auto& monster : monsters) {
            int distance = (hero.position - monster->position).distance();
            int turns_to_reach = countTurnsToInterdict(*monster, hero);
            cerr << elapsed() << "Determined it will take hero " << hero.id << " " << turns_to_reach << " turns to reach monster " << monster->id << endl;
            cerr << elapsed() << "And this monster (" << &monster << ") will have " << monster->healthInNTurns(turns_to_reach) << " health when the hero reaches it." << endl;
            if (!monster->isAliveInNTurns(turns_to_reach))
                continue;

            if (closest == nullptr || distance < min_distance) {
                closest = monster;
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

    map<int, vector<Point>> findBestLocationsToPlaceDefenders(Segment<vector<Entity*>>& segment, Ring<vector<Entity*>>& ring) {
        const int attack_squared = ATTACK_RADIUS * ATTACK_RADIUS;

        int ringsToTrack = 6;
        int ringDistance = (ring.outerDistance - ring.innerDistance) / ringsToTrack;
        RingList<int> reachableMonsters(center, ring.innerDistance, ringDistance, ringsToTrack, 6, segment.startAngle, segment.endAngle);

        cerr << elapsed() << "Finding points that reach the most monsters" << endl;

        for (Ring<int>& ring : reachableMonsters.rings) {
            //cerr << elapsed() << "  Examining ring" << endl;
            for (Segment<int>& s : ring.segmentList) {
                //cerr << elapsed() << "    Examining segment" << endl;
                for (const Entity* e : threats) {
                    Delta distance = e->position - s.center;
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
        cerr << "Base is top left corner" << endl;
        start_angle = 0;
        end_angle = M_PI_2;
    }
    else if (base.x == RIGHT_EDGE && base.y == 0) {
        cerr << "Base is top right corner" << endl;
        start_angle = M_PI_2;
        end_angle = M_PI;
    }
    else if (base.x == RIGHT_EDGE && base.y == BOTTOM_EDGE) {
        cerr << "Base is bottom right corner" << endl;
        start_angle = M_PI;
        end_angle = M_PI * 3.0 / 2.0;
    }
    else if (base.x == 0 && base.y == BOTTOM_EDGE) {
        cerr << "Base is bottom left corner" << endl;
        start_angle = M_PI * 3.0 / 2.0;
        end_angle = M_PI * 2.0;
    }
    else {
        cerr << "Base is not in corner" << endl;
        start_angle = 0.0;
        end_angle = M_PI * 2.0;
    }

    cerr << "Base defense start angle: " << radiansToDegrees(start_angle) << ", end angle: " << radiansToDegrees(end_angle) << endl;

    double defender_arc = (end_angle - start_angle) / heroes_per_player;
    double current = start_angle + defender_arc / 2.0;
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

    int turn_count = 0;
    // game loop
    while (1) {
        turn_count++;
        for (int i = 0; i < 2; i++) {
            int health; // Each player's base health
            int mana; // Ignore in the first league; Spend ten mana to cast a spell
            cin >> health >> mana; cin.ignore();
            if (i == 0) {
                start = Clock::now();
                cerr << elapsed() << "Turn: " << turn_count << endl;
            }
            cerr << "Base Health: " << health << ", Mana: " << mana << endl;
        }
        int entity_count; // Amount of heros and monsters you can see
        cin >> entity_count; cin.ignore();

        cerr << elapsed() << "Entity Count: " << entity_count << endl;
        vector<Entity> entities(entity_count);

        EntityThreatList threatList(base, BASE_RADIUS, RING_SIZE, 3, heroes_per_player, start_angle, end_angle);

        cerr << elapsed() << "Reading entities" << endl;
        for (int i = 0; i < entity_count; i++) {
            Entity& entity = entities[i];
            cin >> entity;
            entity.end_polar = Polar(base, entity.end_position);
            if (entity.targettingMyBase() || entity.willTargetMyBase())
                cerr << elapsed() << "Entity[" << i << "]: " << entity << endl;
            //cerr << elapsed() << "Placing entity " << i << endl;
            threatList.placeEntity(&entity);
            //cerr << elapsed() << "Entity placed " << endl;
        }
        cerr << elapsed() << "Read entities" << endl;

        vector<unique_ptr<Action>> actions = threatList.determineActions();

        cerr << elapsed() << "Actions determined" << endl;

        for (auto& action : actions) {
            cout << action->toString() << endl;
        }
    }
}