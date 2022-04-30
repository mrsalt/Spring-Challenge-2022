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
const int BASE_DAMAGE_RADIUS = 300;
const int RING_SIZE = 2400;
const int ATTACK_RADIUS = 800;
const int ATTACK_SQUARED = ATTACK_RADIUS * ATTACK_RADIUS;
const int HERO_TRAVEL = 800;
const int MONSTER_TRAVEL = 400;
const int HERO_TRAVEL_SQUARED = HERO_TRAVEL * HERO_TRAVEL;
const int HERO_DAMAGE = 2; // heroes do 2 points of damage to nearby enemies
const int WIND_SPELL_RANGE = 1280;
const int WIND_DISTANCE = 2200;

const int RIGHT_EDGE = 17630;
const int BOTTOM_EDGE = 9000;
int heroes_per_player; // Always 3

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

    static Point middle(const Point& a, const Point& b) {
        return { (a.x + b.x) / 2, (a.y + b.y) / 2 };
    }

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
vector<Point> default_attack_position;

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

    Polar rotate(double rad) const {
        double new_theta = theta + rad;
        if (new_theta < 0)
            new_theta += 2.0 * M_PI;
        else if (new_theta > 2.0 * M_PI)
            new_theta -= 2.0 * M_PI;
        return Polar(dist, new_theta);
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
    Polar polar;
    int shield_life; // Ignore for this league; Count down until shield spell fades
    int is_controlled; // Ignore for this league; Equals 1 when this entity is under a control spell
    int health; // Remaining health of this monster
    vector<int> adjusted_health;
    Delta velocity; // Trajectory of this monster
    int near_base; // 0=monster with no target yet, 1=monster targeting a base
    ThreatType threat_for; // Given this monster's trajectory, is it a threat to 1=your base, 2=your opponent's base, 0=neither

    Point end_position;

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
        cerr << elapsed() << "Hero " << hero.id << " will cause damage to monster " << id << ".  Current health: " << health << endl;
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

    Point futureEndPosition(int turns) const {
        return position + (velocity * (turns + 1));
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
        out << spacer << "polar: " << c.polar << "\n";
        out << spacer << "shield_life: " << c.shield_life << "\n";
        out << spacer << "health: " << c.health << "\n";
        out << spacer << "is_controlled: " << c.is_controlled << "\n";
        out << spacer << "velocity: " << c.velocity << "\n";
        out << spacer << "threat_for: " << c.threat_for << "\n";
        out << spacer << "end_position: " << c.end_position << "\n";
        out << "}";
        return out;
    }
};

struct PlayerStats {
    int health;
    int mana;
    Point base;
    double start_angle;
    double end_angle;

    friend istream& operator >> (istream& in, PlayerStats& c) {
        in >> c.health >> c.mana; cin.ignore();
        return in;
    }

    friend ostream& operator << (ostream& out, const PlayerStats& c) {
        out << "Base Health: " << c.health << ", Mana: " << c.mana;
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
    int threatCount;
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

struct ActionCalculator : RingList<vector<Entity*>> {

    ActionCalculator(const PlayerStats& my_stats, const PlayerStats& opponent_stats, int initialDistance, int ringDistance, int ringsToTrack, int segments, double startAngle, double endAngle)
        : RingList<vector<Entity*>>(my_stats.base, initialDistance, ringDistance, ringsToTrack, segments, startAngle, endAngle)
        , my_stats(my_stats)
        , opponent_stats(opponent_stats)
    {
        cerr << elapsed() << "Initializing threat list.  Start angle: " << radiansToDegrees(startAngle) << ", End angle: " << radiansToDegrees(endAngle) << endl;
    }

    vector<Entity*> threats;
    vector<Entity*> heroes;
    vector<Entity*> monsters;
    const PlayerStats my_stats;
    const PlayerStats opponent_stats;
    vector<pair<Entity*, int>> turnsToReachBase;
    int turnsUntilILose;

    void placeEntity(Entity* e) {
        if (e->type == EntityType::Hero) {
            heroes.push_back(e);
        }
        else if (e->type == EntityType::OpponentHero) {
        }
        else {
            monsters.push_back(e);
            if (e->targettingMyBase() || e->willTargetMyBase()) {
                threats.push_back(e);
            }
            for (int i = 0; i < rings.size(); i++) {
                if (e->polar.dist < rings[i].outerDistance) {
                    auto& ring = rings[i];
                    cerr << elapsed() << "Monster " << e->id << " (distance: " << e->polar.dist << ") in ring " << i << " (distance: " << ring.innerDistance << " - " << ring.outerDistance << ")" << endl;
                    int segmentNum = (e->polar.theta - startAngle) / segmentArc;
                    //cerr << elapsed() << "  segment #: " << segmentNum << ", startAngle: " << radiansToDegrees(startAngle) << ", e->polar: " << e->polar << ", segment arc: " << radiansToDegrees(segmentArc) << endl;
                    auto& segment = ring.segmentList[segmentNum];
                    segment.data.push_back(e);
                    if (e->targettingMyBase() || e->willTargetMyBase())
                        ring.threatCount++;
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

    int calculateTurnsToReachBase(const Entity& threat) {
        if (threat.targettingMyBase()) {
            return (threat.polar.dist - BASE_DAMAGE_RADIUS) / MONSTER_TRAVEL;
        }
        Point position = threat.position;
        for (int t = 0; t < 50; t++) {
            Polar p(center, position);
            if (p.dist < BASE_RADIUS) { // now monster will target base
                return t + (p.dist - BASE_DAMAGE_RADIUS) / MONSTER_TRAVEL;
            }
            position += threat.velocity;
        }
        cerr << "Error: \"threat\" " << threat.id << " will not reach base in 50 turns.  Will target my base: " << threat.willTargetMyBase() << endl;
        throw exception();
    }

    void calculateTurnsUntilILose() {
        if (my_stats.health > threats.size()) {
            turnsUntilILose = 50;
            return;
        }
        for (auto threat : threats) {
            turnsToReachBase.push_back(make_pair(threat, calculateTurnsToReachBase(*threat)));
        }
        sort(begin(turnsToReachBase), end(turnsToReachBase), [](const auto& a, const auto& b) -> bool { return a.second < b.second; });
        cerr << elapsed() << "Turns to reach base: " << endl;
        for (pair<Entity*, int> p : turnsToReachBase) {
            cerr << elapsed() << "  Entity ID: " << p.first->id << ", Turns: " << p.second << endl;
        }
        turnsUntilILose = turnsToReachBase[my_stats.health - 1].second;
    }

    bool windAttackIsPossible(vector<unique_ptr<Action>>& actions) {
        if (my_stats.mana < 30 * opponent_stats.health)
            return false;

        const int wind_attack_distance = WIND_DISTANCE * 3 + 200;
        // can I cast WIND right now and send monsters in?
        map<const Entity*, vector<int>> blow_distance;

        for (int h = 0; h < heroes.size(); h++) {
            const Entity& hero = *heroes[h];
            for (auto opponent_threat : monsters) {
                Polar p(opponent_stats.base, opponent_threat->position);
                if (p.dist < wind_attack_distance) {
                    if ((hero.position - opponent_threat->position).distance() < WIND_SPELL_RANGE) {
                        blow_distance[opponent_threat].push_back(h);
                    }
                }
            }
        }

        if (!blow_distance.empty()) {
            for (auto& pair : blow_distance) {
                if (pair.second.size() == heroes.size()) {
                    for (int h : pair.second) {
                        cerr << elapsed() << "Casting wind attack!!" << endl;
                        actions[h] = make_unique<WindSpellAction>(opponent_stats.base);
                    }
                    return true;
                }
            }
        }

        // are there monsters clustered together (or not) that I can push towards my opponents base?
        int turns_to_project = 5;

        for (int t = 0; t < turns_to_project; t++) {
            vector<Entity*> soccer_balls;
            for (auto opponent_threat : monsters) {
                Polar p(opponent_stats.base, opponent_threat->futurePosition(t));
                if (p.dist < wind_attack_distance && opponent_threat->health > t* HERO_DAMAGE) {
                    soccer_balls.push_back(opponent_threat);
                }
            }
            // are there any soccer balls that all 3 of my heroes can get close to the 'end' position of in 't' turns?
            vector<Entity*> reachable_balls;
            for (auto ball : soccer_balls) {
                Point endPosition = ball->futureEndPosition(t);
                bool reachable = true;
                for (int h = 0; h < heroes.size(); h++) {
                    int toTravel = ((heroes[h]->position - endPosition).distance() - WIND_SPELL_RANGE);
                    if (toTravel > (t + 1)* HERO_TRAVEL) {
                        reachable = false;
                        break;
                    }
                }
                if (reachable) {
                    reachable_balls.push_back(ball);
                }
            }
            if (reachable_balls.size() > 1) {
                // if we can reach more than 1, let's find the 2 that are closest to reach other.
                pair<Entity*, Entity*> closest = { nullptr, nullptr };
                int min_distance;
                for (auto a : reachable_balls) {
                    for (auto b : reachable_balls) {
                        if (a == b)
                            continue;
                        int dist = (a->futureEndPosition(t) - b->futureEndPosition(t)).distance();
                        if (closest.first == nullptr || dist < min_distance) {
                            min_distance = dist;
                            closest.first = a;
                            closest.second = b;
                        }
                    }
                }
                // We can get two in one shot... maybe.
                if (min_distance < (WIND_SPELL_RANGE * 2) - 50) {
                    Point a = closest.first->futureEndPosition(t);
                    Point b = closest.second->futureEndPosition(t);
                    Point c = Point::middle(a, b);
                    Polar v(c, a);
                    Polar v1 = v.rotate(M_PI_2);
                    Polar v2 = v.rotate(-M_PI_2);
                    Point p1 = findNearestPointAlongVectorExceedingDistanceToTarget(v1, c, a, 1200);
                    Point p2 = findNearestPointAlongVectorExceedingDistanceToTarget(v2, c, a, 1200);
                    Point target = maxDistance(p1, heroes) < maxDistance(p2, heroes) ? p1 : p2;
                    // Hopefully the monsters are not killed along the way to get close to them!
                    cerr << elapsed() << "Moving heroes to " << target << " to prepare for a WIND action affecting " << closest.first->id << " and " << closest.second->id << " in " << t << " turn(s)." << endl;
                    for (int h = 0; h < heroes.size(); h++)
                        actions[h] = make_unique<MoveAction>(target);
                    return true;
                }
            }
            else if (reachable_balls.size() > 0) {
                auto ball = reachable_balls[0];
                Point target = ball->futureEndPosition(t);
                for (int h = 0; h < heroes.size(); h++) {
                    auto hero = heroes[h];
                    Polar v(target, hero->position);
                    Point p1 = findNearestPointAlongVectorExceedingDistanceToTarget(v, target, hero->position, 1200);
                    // Don't kill the ball early!  Back off if we're too close.
                    int dist = (p1 - ball->position).distance();
                    cerr << elapsed() << "Distance for hero " << h << " target (" << p1 << ") to " << ball->id << ":  " << dist << endl;
                    if (dist < (ATTACK_RADIUS + 10)) {
                        Polar v(ball->position, hero->position);
                        p1 = findNearestPointAlongVectorExceedingDistanceToTarget(v, ball->position, hero->position, 1200);
                    }
                    cerr << elapsed() << "Moving hero to " << p1 << " to prepare for a WIND action affecting " << ball->id << " in " << t << " turn(s)." << endl;
                    actions[h] = make_unique<MoveAction>(p1);
                }
                return true;
            }
        }

        cerr << elapsed() << "No known enemies to WIND attack currently, moving towards SCOUT positions" << endl;
        int minDist = -1;
        for (int h_offset = 0; h_offset < heroes.size(); h_offset++) {
            for (int p_offset = 0; p_offset < default_attack_position.size(); p_offset++) {
                int h1 = (0 + h_offset) % 3;
                int h2 = (1 + h_offset) % 3;
                int h3 = (2 + h_offset) % 3;
                Point p1 = default_attack_position[(0 + p_offset) % 3];
                Point p2 = default_attack_position[(1 + p_offset) % 3];
                Point p3 = default_attack_position[(2 + p_offset) % 3];
                int d[] = { (heroes[h1]->position - p1).squared(), (heroes[h2]->position - p2).squared(), (heroes[h3]->position - p3).squared() };
                int maxD = *max_element(begin(d), end(d));
                if (minDist == -1 || maxD < minDist) {
                    actions[h1] = make_unique<MoveAction>(p1);
                    actions[h2] = make_unique<MoveAction>(p2);
                    actions[h3] = make_unique<MoveAction>(p3);
                }
            }
        }
        return true;
    }

    int maxDistance(const Point& p, const vector<Entity*>& entities) {
        int maxD = 0;
        for (auto e : entities) {
            int dist = (e->position - p).distance();
            if (dist > maxD) {
                maxD = dist;
            }
        }
        return maxD;
    }

    Point findNearestPointAlongVectorExceedingDistanceToTarget(Polar& vector, Point& center, Point& target, int distance)
    {
        for (int d = 10; d < distance * 2; d += 10) {
            Point candidate = Polar(d, vector.theta).toPoint(center);
            if ((candidate - target).distance() > distance) {
                return candidate;
            }
        }
        cerr << elapsed() << "Unable to find nearest point!  (Line " << __LINE__ << ")" << endl;
        throw exception();
    }

    vector<unique_ptr<Action>> determineActions() {
        cerr << elapsed() << "Determining actions to take" << endl;

        calculateTurnsUntilILose();
        cerr << elapsed() << "Turns until I lose (without attacking threats): " << turnsUntilILose << endl;

        vector<unique_ptr<Action>> actions(heroes.size());

        if (windAttackIsPossible(actions)) {
            return actions;
        }

        if (monsters.empty()) {
            cerr << elapsed() << "No monsters.  Ordering all heroes to default locations." << endl;
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
            cerr << elapsed() << "And this monster will have " << monster->healthInNTurns(turns_to_reach) << " health when the hero reaches it." << endl;
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

        int ringsToTrack = 8;
        int segments = 8;
        int ringDistance = (ring.outerDistance - ring.innerDistance) / ringsToTrack;
        RingList<int> reachableMonsters(center, ring.innerDistance, ringDistance, ringsToTrack, segments, segment.startAngle, segment.endAngle);

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

void init_top_left(PlayerStats& stats) {
    stats.start_angle = 0;
    stats.end_angle = M_PI_2;
    stats.base = { 0, 0 };
}

void init_top_right(PlayerStats& stats) {
    stats.start_angle = M_PI_2;
    stats.end_angle = M_PI;
    stats.base = { RIGHT_EDGE, 0 };
}

void init_bottom_right(PlayerStats& stats) {
    stats.start_angle = M_PI;
    stats.end_angle = M_PI * 3.0 / 2.0;
    stats.base = { RIGHT_EDGE, BOTTOM_EDGE };
}

void init_bottom_left(PlayerStats& stats) {
    stats.start_angle = M_PI * 3.0 / 2.0;
    stats.end_angle = M_PI * 2.0;
    stats.base = { 0, BOTTOM_EDGE };
}


void find_default_hero_placements(PlayerStats& my_stats, PlayerStats& opponent_stats)
{
    if (my_stats.base.x == 0 && my_stats.base.y == 0) {
        cerr << "Base is top left corner" << endl;
        init_top_left(my_stats);
        init_bottom_right(opponent_stats);
    }
    else if (my_stats.base.x == RIGHT_EDGE && my_stats.base.y == 0) {
        cerr << "Base is top right corner" << endl;
        init_top_right(my_stats);
        init_bottom_left(opponent_stats);
    }
    else if (my_stats.base.x == RIGHT_EDGE && my_stats.base.y == BOTTOM_EDGE) {
        cerr << "Base is bottom right corner" << endl;
        init_bottom_right(my_stats);
        init_top_left(opponent_stats);
    }
    else if (my_stats.base.x == 0 && my_stats.base.y == BOTTOM_EDGE) {
        cerr << "Base is bottom left corner" << endl;
        init_bottom_left(my_stats);
        init_top_right(opponent_stats);
    }
    else {
        cerr << "Base is not in corner!" << endl;
        throw exception();
    }

    cerr << "Base defense start angle: " << radiansToDegrees(my_stats.start_angle) << ", end angle: " << radiansToDegrees(my_stats.end_angle) << endl;

    double defender_arc = (my_stats.end_angle - my_stats.start_angle) / heroes_per_player;
    double current = my_stats.start_angle + defender_arc / 2.0;
    for (int i = 0; i < heroes_per_player; i++) {
        Point p = Polar(BASE_RADIUS + HERO_TRAVEL, current).toPoint(my_stats.base);
        cerr << "Default hero position[" << i << "]: " << p << endl;
        default_hero_location.push_back(p);
        current += defender_arc;
    }

    current = opponent_stats.start_angle + defender_arc;
    for (int i = 0; i < heroes_per_player; i++) {
        Point p = Polar(BASE_RADIUS + HERO_TRAVEL * 3, current).toPoint(opponent_stats.base);
        default_attack_position.push_back(p);
        current += defender_arc / 2;
    }
}

int main()
{
    start = Clock::now();
    PlayerStats my_stats;
    PlayerStats opponent_stats;
    cin >> my_stats.base; cin.ignore();
    cerr << elapsed() << "Base: " << my_stats.base << endl;

    cin >> heroes_per_player; cin.ignore();
    cerr << elapsed() << "Heroes: " << heroes_per_player << endl;

    find_default_hero_placements(my_stats, opponent_stats);

    int turn_count = 0;
    // game loop
    while (1) {
        turn_count++;
        cin >> my_stats;
        start = Clock::now();
        cerr << elapsed() << "Turn: " << turn_count << endl;
        cin >> opponent_stats;
        cerr << elapsed() << "My Stats: " << my_stats << endl;
        cerr << elapsed() << "Opponent Stats: " << opponent_stats << endl;

        int entity_count; // Amount of heros and monsters you can see
        cin >> entity_count; cin.ignore();

        cerr << elapsed() << "Entity Count: " << entity_count << endl;
        vector<Entity> entities(entity_count);

        ActionCalculator calculator(my_stats, opponent_stats, BASE_RADIUS, RING_SIZE, 3, heroes_per_player, my_stats.start_angle, my_stats.end_angle);

        cerr << elapsed() << "Reading entities" << endl;
        for (int i = 0; i < entity_count; i++) {
            Entity& entity = entities[i];
            cin >> entity;
            entity.polar = Polar(my_stats.base, entity.position);

            //if (entity.targettingMyBase() || entity.willTargetMyBase())
            //    cerr << elapsed() << "Entity[" << i << "]: " << entity << endl;

            //cerr << elapsed() << "Placing entity " << i << endl;
            calculator.placeEntity(&entity);
            //cerr << elapsed() << "Entity placed " << endl;
        }
        //cerr << elapsed() << "Read entities" << endl;

        vector<unique_ptr<Action>> actions = calculator.determineActions();

        //cerr << elapsed() << "Actions determined" << endl;

        for (auto& action : actions) {
            cout << action->toString() << endl;
        }
    }
}