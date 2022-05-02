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

const int RING_SIZE = 5000;
const int RINGS_TO_TRACK = 3;
int turn_count = 0;
bool inAttackMode = false;

const int BASE_RADIUS = 5000;
const int BASE_DAMAGE_RADIUS = 300;
const int ATTACK_RADIUS = 800;
const int ATTACK_SQUARED = ATTACK_RADIUS * ATTACK_RADIUS;
const int HERO_TRAVEL = 800;
const int MONSTER_TRAVEL = 400;
const int HERO_TRAVEL_SQUARED = HERO_TRAVEL * HERO_TRAVEL;
const int HERO_DAMAGE = 2; // heroes do 2 points of damage to nearby enemies
const int WIND_SPELL_RANGE = 1280;
const int WIND_DISTANCE = 2200;
const int CONTROL_SPELL_RANGE = 2200;
const int SHIELD_SPELL_RANGE = 2200;
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
vector<Point> default_hero_location_in_standard_mode;
vector<Point> default_hero_location_in_attack_mode;
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

struct Arc
{
    double start;
    double end;

    bool includes(const Polar& p) const {
        return p.theta >= start && p.theta <= end;
    }
};
vector<Arc> permitted_range;

struct PlayerStats {
    int health;
    int mana;
    Point base;
    Arc arc;

    friend istream& operator >> (istream& in, PlayerStats& c) {
        in >> c.health >> c.mana; cin.ignore();
        return in;
    }

    friend ostream& operator << (ostream& out, const PlayerStats& c) {
        out << "Base Health: " << c.health << ", Mana: " << c.mana;
        return out;
    }
};

PlayerStats my_stats;
PlayerStats opponent_stats;

struct Entity
{
    int id; // Unique identifier
    EntityType type; // 0=monster, 1=your hero, 2=opponent hero
    Point position; // Position of this entity
    Polar polar;
    int shield_life; // Count down until shield spell fades
    int is_controlled; // Equals 1 when this entity is under a control spell
    int health; // Remaining health of this monster
    vector<int> adjusted_health;
    Delta velocity; // Trajectory of this monster
    int near_base; // 0=monster with no target yet, 1=monster targeting a base
    ThreatType threat_for; // Given this monster's trajectory, is it a threat to 1=your base, 2=your opponent's base, 0=neither

    Point end_position;

    vector<int> damagedBy;
    bool closerToMyBase;

    bool targettingMyBase() const {
        return type == EntityType::Monster && near_base == 1 && closerToMyBase;
    }

    bool willTargetMyBase() const {
        return type == EntityType::Monster && near_base == 0 && threat_for == ThreatType::MyBase;
    }

    bool targettingOpponentBase() const {
        return type == EntityType::Monster && near_base == 1 && !closerToMyBase;
    }

    bool willTargetOpponentBase() const {
        return type == EntityType::Monster && near_base == 0 && threat_for == ThreatType::OpponentBase;
    }

    void damage(const Entity& hero, int turnsToReach) {
        // Each hero can "damage" a given entity only once.  This method computes all the damage a hero will do to a given monster.
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
        //cerr << elapsed() << "  Projected health: " << endl;
        //for (int i = 0; i < adjusted_health.size(); i++) {
        //    cerr << elapsed() << "    turn[" << i << "] = " << adjusted_health[i] << endl;
        //}
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
        c.closerToMyBase = (c.position - my_stats.base).squared() < (c.position - opponent_stats.base).squared();
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

enum class ActionType {
    WAIT,
    MOVE,
    WIND,
    CONTROL,
    SHIELD
};

class Action
{
public:
    const ActionType type;

    Action(ActionType t)
        : type(t) {}
    virtual string toString() = 0;
};

class WaitAction : public Action
{
public:
    WaitAction() : Action(ActionType::WAIT) {}
    string toString() { return "WAIT"; }
};

class MoveAction : public Action
{
public:
    const Point location;
    MoveAction(const Point& p) : Action(ActionType::MOVE), location(p) {}
    string toString() { return "MOVE " + location.toString(); }
};

class WindSpellAction : public Action
{
public:
    const Point target;
    WindSpellAction(const Point& target) : Action(ActionType::WIND), target(target) {}
    string toString() { return "SPELL WIND " + target.toString(); }
};

class ShieldSpellAction : public Action
{
public:
    const int targetId;
    ShieldSpellAction(const Entity& target) : Action(ActionType::SHIELD), targetId(target.id) {}
    string toString() { return "SPELL SHIELD " + to_string(targetId); }
};


class ControlSpellAction : public Action
{
public:
    const int targetId;
    const Point location;
    ControlSpellAction(const Entity& target, const Point& location)
        : Action(ActionType::CONTROL)
        , targetId(target.id)
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
    T data;
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
    vector<Entity*> opposing_heroes;
    const PlayerStats my_stats;
    const PlayerStats opponent_stats;
    vector<pair<Entity*, int>> turnsToReachBase;
    vector<Entity*> top_threats;
    int turnsUntilBaseIsAttacked;

    void placeEntity(Entity* e) {
        if (e->type == EntityType::Hero) {
            heroes.push_back(e);
        }
        else if (e->type == EntityType::OpponentHero) {
            opposing_heroes.push_back(e);
        }
        else {
            monsters.push_back(e);
            if (e->targettingMyBase() || e->willTargetMyBase()) {
                threats.push_back(e);
            }
            for (int i = 0; i < rings.size(); i++) {
                if (e->polar.dist < rings[i].outerDistance) {
                    auto& ring = rings[i];
                    ring.data.push_back(e);
                    int segmentNum = (e->polar.theta - startAngle) / segmentArc;
                    cerr << elapsed() << "Monster " << e->id << " (distance: " << e->polar.dist << ") in ring " << i << " (distance: " << ring.innerDistance << " - " << ring.outerDistance << "), segment: " << segmentNum << endl;
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

    pair<bool, vector<int>> canThreatBeEliminated(const Entity& threat, const vector<unique_ptr<Action>>& actions) {
        Point position = threat.position;
        int health = threat.health;
        vector<pair<int, int>> turnsToInterdict;
        cerr << elapsed() << "Turns to interdict:" << threat.id << endl;
        for (int h = 0; h < heroes.size(); h++) {
            if (actions[h])
                continue; // action already assigned
            // Don't allow heroes outside their permitted range
            if ((threat.position - my_stats.base).distance() > BASE_RADIUS && !permitted_range[h].includes(threat.polar))
                continue;
            int turns = countTurnsToInterdict(threat, *heroes[h]);
            cerr << elapsed() << "  Hero[" << h << "]: " << turns << ", distance: " << (heroes[h]->position - threat.position).distance() << endl;
            turnsToInterdict.push_back(make_pair(h, turns));
        }
        vector<int> heroes_needed_for_threat;

        if (turnsToInterdict.empty()) // all heroes assigned already
            return make_pair(false, heroes_needed_for_threat);

        sort(begin(turnsToInterdict), end(turnsToInterdict), [](const auto& a, const auto& b) -> bool {
            return a.second < b.second;
            });

        for (int turn = 0; turn < 50; turn++) {
            for (pair<int, int> pair : turnsToInterdict) {
                if (pair.second <= turn) {
                    health -= HERO_DAMAGE;
                    //cerr << elapsed() << "Hero " << pair.first << " causes damage on turn " << turn << ", monster health: " << health << endl;
                    if (pair.second == turn) {
                        heroes_needed_for_threat.push_back(pair.first);
                        if (threat.shield_life == 0) {
                            cerr << elapsed() << "Monster not shielded -- using 1 hero only." << endl;
                            break; // if the monster isn't shielded, one hero can take care of it by using spells
                        }
                    }
                }
            }
            if (health <= 0 || (threat.shield_life == 0 && !heroes_needed_for_threat.empty()))
                return make_pair(true, heroes_needed_for_threat);
            position += threat.velocity;
            if ((position - center).distance() <= 300)
                return make_pair(false, heroes_needed_for_threat);
        }
        cerr << "Unable to determine if threat can be eliminated " << threat << endl;
        throw exception();
    }

    Point moveTowards(const Point& position, const Point& p) {
        Polar v(position, p);
        if (v.dist > HERO_TRAVEL)
            v.dist = HERO_TRAVEL;
        return v.toPoint(position);
    }

    Point moveTowards(const Entity& hero, const Point& p) {
        return moveTowards(hero.position, p);
    }

    unique_ptr<Action> assignMoveAction(const Entity& hero, const Point& p) {
        Delta d = p - hero.position;
        int distance = d.distance();

        //Polar v(hero.position, p);
        //v.dist = HERO_TRAVEL;
        //Delta velocity = v.toDelta();

        // How many turns until it can damage this location?
        int turns_to_damage = distance / HERO_TRAVEL;

        //if (hero.position != p) {
        //    cerr << elapsed() << "hero moving from " << hero.position << " to " << p << endl;
        //    cerr << elapsed() << "turns_to_damage(" << turns_to_damage << ") = distance(" << distance << ") / HERO_TRAVEL(" << HERO_TRAVEL << ")" << endl;
        //}

        for (Entity* threat : threats) {
            Point actualLocation = hero.position;

            for (int t = 0; t < turns_to_damage + 1; t++) {
                if (t == turns_to_damage)
                    actualLocation = p;
                else {
                    // hero will move towards this location, but will not reach it this turn.
                    // where will it reach this turn?
                    actualLocation = moveTowards(actualLocation, p);
                }

                Point fp = threat->futurePosition(t);
                int distanceToThreat = (fp - actualLocation).distance();
                //cerr << elapsed() << "threat future position[" << t << "] = " << fp << ", hero location = " << actualLocation << ", distanceToThreat = " << distanceToThreat << endl;
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

    void calculateTurnsUntilIMonstersAttack() {

        // sort opposing heroes by proximity to my base.
        sort(begin(opposing_heroes), end(opposing_heroes), [my_stats = my_stats](const auto& a, const auto& b) -> bool {
            int aDistance = (my_stats.base - a->position).distance();
            int bDistance = (my_stats.base - b->position).distance();
            return aDistance < bDistance;
            });

        if (threats.empty()) {
            turnsUntilBaseIsAttacked = 50;
            return;
        }
        for (auto threat : threats) {
            turnsToReachBase.push_back(make_pair(threat, calculateTurnsToReachBase(*threat)));
        }
        sort(begin(turnsToReachBase), end(turnsToReachBase), [](const auto& a, const auto& b) -> bool { return a.second < b.second; });
        cerr << elapsed() << "Turns to reach base: " << endl;
        for (pair<Entity*, int> p : turnsToReachBase) {
            top_threats.push_back(p.first);
            cerr << elapsed() << "  Threat ID: " << p.first->id << ", Turns: " << p.second << endl;
        }
        turnsUntilBaseIsAttacked = turnsToReachBase[0].second;
        cerr << elapsed() << "Turns until my base is damaged: " << turnsUntilBaseIsAttacked << endl;
    }

    bool windAttackIsPossible(vector<unique_ptr<Action>>& actions) {
        if (my_stats.mana < 30 * opponent_stats.health)
            return false;

        const int wind_attack_distance = WIND_DISTANCE * 3 + 200;
        // can I cast WIND right now and send monsters in?
        map<const Entity*, vector<int>> blow_distance;

        for (auto opponent_threat : monsters) {
            Polar p(opponent_stats.base, opponent_threat->position);
            cerr << elapsed() << "Monster " << opponent_threat->id << " distance to opponent base is " << p.dist << endl;
            if (p.dist < wind_attack_distance) {
                for (int h = 0; h < heroes.size(); h++) {
                    const Entity& hero = *heroes[h];
                    int dist = (hero.position - opponent_threat->position).distance();
                    cerr << elapsed() << "Hero " << h << " at " << hero.position << " distance to monster " << opponent_threat->id << " is " << dist << (dist < WIND_SPELL_RANGE ? " (IN RANGE)" : "") << endl;
                    if (dist < WIND_SPELL_RANGE) {
                        blow_distance[opponent_threat].push_back(h);
                    }
                }
            }
        }

        if (!blow_distance.empty()) {
            for (auto& pair : blow_distance) {
                cerr << elapsed() << pair.second.size() << " heroes can cast wind attack" << endl;
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
        int turns_to_project = 6;

        for (int t = 1; t < turns_to_project; t++) {
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
                Point endPosition = ball->futurePosition(t);
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
                        int dist = (a->futurePosition(t) - b->futurePosition(t)).distance();
                        if (closest.first == nullptr || dist < min_distance) {
                            min_distance = dist;
                            closest.first = a;
                            closest.second = b;
                        }
                    }
                }
                // We can get two in one shot... maybe.
                if (min_distance < (WIND_SPELL_RANGE * 2) - 50) {
                    Point a = closest.first->futurePosition(t);
                    Point b = closest.second->futurePosition(t);
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
                Point target = ball->futurePosition(t);
                for (int h = 0; h < heroes.size(); h++) {
                    auto hero = heroes[h];
                    Polar v(target, hero->position);
                    Point p1 = findNearestPointAlongVectorExceedingDistanceToTarget(v, target, hero->position, 1200);
                    // Don't kill the ball early!  Back off if we're too close.
                    Point endPosition = moveTowards(*hero, p1);
                    int dist = (endPosition - ball->position).distance();
                    cerr << elapsed() << "Distance for hero " << h << " target (" << p1 << ") to " << ball->id << ":  " << dist << endl;
                    if (dist < (ATTACK_RADIUS + 10)) {
                        Polar v(ball->position, hero->position);
                        p1 = findNearestPointAlongVectorExceedingDistanceToTarget(v, ball->position, hero->position, 1200);
                    }
                    cerr << elapsed() << "Moving hero " << h << " to " << p1 << " for a WIND action affecting " << ball->id << " in " << t << " turn(s).  (dist: " << (p1 - ball->position).distance() << ")" << endl;
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

    Point findNearestPointAlongVectorExceedingDistanceToTarget(Polar& vector, Point& center, Point& target, int distance) {
        for (int d = 10; d < distance * 2; d += 10) {
            Point candidate = Polar(d, vector.theta).toPoint(center);
            cerr << "Distance " << d << " from " << center << " using angle " << radiansToDegrees(vector.theta) << " is " << candidate << endl;
            if ((candidate - target).distance() > distance) {
                return candidate;
            }
        }
        cerr << elapsed() << "Unable to find nearest point!  (Line " << __LINE__ << ")" << endl;
        throw exception();
    }

    bool allActionsAreSet(const vector<unique_ptr<Action>>& actions) {
        if (actions.empty())
            return false;
        for (auto& action : actions) {
            if (!action)
                return false;
        }
        return true;
    }

    vector<unique_ptr<Action>> determineActions() {
        cerr << elapsed() << "Determining actions to take" << endl;

        vector<unique_ptr<Action>> actions(heroes.size());

        if (turn_count > 100 && my_stats.mana > 100 && (top_threats.empty() || turnsUntilBaseIsAttacked > 10)) {
            inAttackMode = true;
        }

        default_hero_location = inAttackMode ? default_hero_location_in_attack_mode : default_hero_location_in_standard_mode;

        if (inAttackMode) {
            // goals:
            // 1) control monsters that are > ATTACK_DISTANCE away and have full health.
            // 2) move towards enemy base
            // 3) shield attacking monsters once I reach enemy base
            // 4) move away from monsters attacking enemy base
            const Entity& attacker = *heroes[1];

            unique_ptr<Action> attackerAction = findMonsterToControl(attacker, actions);
            if (!attackerAction) {
                int distToOpponentBase = (attacker.position - opponent_stats.base).distance();
                if (distToOpponentBase > BASE_RADIUS) {
                    // zig zag towards the base
                    double angle_adjust = M_PI_2 / 2.0; // 45 degrees
                    if ((turn_count / 4) % 2 == 0)
                        angle_adjust = angle_adjust * -1.0;
                    Point p = Polar(attacker.position, opponent_stats.base).rotate(angle_adjust).toPoint(attacker.position);
                    cerr << elapsed() << "Attacker targetting " << p << ", angle_adjust: " << radiansToDegrees(angle_adjust) << endl;
                    attackerAction = assignMoveAction(attacker, p);
                }
                else {
                    vector<Entity*> shieldableMonsters;
                    for (auto& monster : monsters) {
                        if ((monster->targettingOpponentBase() || monster->willTargetOpponentBase()) && (attacker.position - monster->position).distance() < SHIELD_SPELL_RANGE && monster->shield_life == 0)
                            shieldableMonsters.push_back(monster);
                    }
                    if (!shieldableMonsters.empty()) {
                        sort(begin(shieldableMonsters), end(shieldableMonsters), [](const auto& a, const auto& b) -> bool {
                            return a->health > b->health;
                            });
                        if (shieldableMonsters[0]->health > 8) {
                            cerr << elapsed() << "Attacker shielding " << shieldableMonsters[0]->id << endl;
                            attackerAction = make_unique<ShieldSpellAction>(*shieldableMonsters[0]);
                        }
                    }
                    if (!attackerAction) {
                        // find a safe spot where I won't attack monsters
                        vector<pair<int, double>> l;
                        for (double r = 0.0; r < M_PI * 2.0; r += M_PI / 32.0) {
                            Point p = Polar(HERO_TRAVEL, r).toPoint(attacker.position);
                            int attackedMonsters = healthyMonstersReachedFrom(p);
                            l.push_back(make_pair(attackedMonsters, r));
                        }
                        sort(begin(l), end(l), [](auto& a, auto& b) -> bool { return a.first < b.first; });
                        int minMonstersAttacked = l[0].first;
                        vector<Point> points;
                        for (auto& p : l) {
                            if (p.first == minMonstersAttacked) {
                                points.push_back(Polar(HERO_TRAVEL, p.second).toPoint(attacker.position));
                            }
                        }
                        sort(begin(points), end(points), [](auto& a, auto& b) -> bool {
                            int aDist = (default_hero_location_in_attack_mode[1] - a).distance();
                            int bDist = (default_hero_location_in_attack_mode[1] - b).distance();
                            return aDist < bDist;
                            });
                        cerr << elapsed() << "Attacker moving to " << points[0] << " to avoid damaging monsters" << endl;
                        attackerAction = assignMoveAction(attacker, points[0]);
                    }
                }
            }
            actions[1] = move(attackerAction);
        }

        if (turnsUntilBaseIsAttacked < 14 && my_stats.mana > 30 && !top_threats.empty()) {
            // which hero is closest to my base?
            vector<pair<int, int>> closest;
            for (int h = 0; h < heroes.size(); h++) {
                if (actions[h])
                    continue;
                int distance = (heroes[h]->position - my_stats.base).distance();
                closest.push_back(make_pair(h, distance));
            }
            sort(begin(closest), end(closest), [](const auto& a, const auto& b) -> bool { return a.second < b.second; });

            // If the monster closest to the base is not shielded and is within distance of the closest hero, cast wind.
            if (top_threats[0]->shield_life == 0 && (top_threats[0]->position - heroes[closest[0].first]->position).distance() < WIND_SPELL_RANGE) {
                cerr << elapsed() << "Assigning wind action for defensive purposes" << endl;
                actions[closest[0].first] = make_unique<WindSpellAction>(opponent_stats.base);
            }
        }

        /*
        if (!opposing_heroes.empty()) {
            auto opposing_hero = *opposing_heroes[0];
            if ((opposing_hero.position - my_stats.base).distance() < BASE_RADIUS) {
                bool wind_found = false;
                for (int h = 0; h < heroes.size(); h++) {
                    if (actions[h])
                        continue;
                    auto hero = *heroes[h];
                    if ((hero.position - opposing_hero.position).distance() < WIND_SPELL_RANGE) {
                        actions[h] = make_unique<WindSpellAction>(opponent_stats.base);
                        wind_found = true;
                        break;
                    }
                }
                if (!wind_found) {
                    vector<int> ids = { 0, 1, 2 };
                    sort(begin(ids), end(ids), [&opposing_hero, heroes=heroes](int a, int b) -> bool {
                        int aDistance = (opposing_hero.position - heroes[a]->position).distance();
                        int bDistance = (opposing_hero.position - heroes[b]->position).distance();
                        return aDistance < bDistance;
                        });
                    cerr << elapsed() << "Moving towards enemy hero to kick him out" << endl;
                    actions[ids[0]] = assignMoveAction(*heroes[ids[0]], opposing_hero.position);
                }
            }
        }
        */

        if (allActionsAreSet(actions)) return actions;

        // Shield against an enemy spellcaster
        /*
        if (opponent_stats.mana > 10 && my_stats.mana > 100 && turn_count > 100) {
            for (int h = 0; h < heroes.size(); h++) {
                auto hero = heroes[h];
                if (hero->shield_life > 0 || (hero->position - my_stats.base).distance() > (BASE_RADIUS + 1000))
                    continue;
                for (int j = 0; j < opposing_heroes.size(); j++) {
                    auto opponent = opposing_heroes[j];
                    if ((opponent->position - hero->position).distance() < CONTROL_SPELL_RANGE + HERO_TRAVEL) {
                        actions[h] = make_unique<ShieldSpellAction>(*hero);
                    }
                }
            }
        }*/

        if (allActionsAreSet(actions)) return actions;

        //if (windAttackIsPossible(actions)) {
        //    return actions;
        //}

        if (monsters.empty()) {
            cerr << elapsed() << "No monsters.  Ordering all heroes to default locations." << endl;
            for (int h = 0; h < heroes.size(); h++) {
                if (!actions[h])
                    actions[h] = assignMoveAction(*heroes[h], default_hero_location[h]);
            }
            return actions;
        }

        for (auto& t : top_threats) {
            const Entity& threat = *t;
            const auto& analysis = canThreatBeEliminated(threat, actions);
            if (analysis.first) {
                cerr << elapsed() << "Ordering required heroes (" << analysis.second.size() << ") to interdict threat " << threat.id << ":" << endl;
                for (int h : analysis.second) {
                    const Entity& hero = *heroes[h];
                    const Point& p = findInterdictionPoint(hero, threat);
                    cerr << elapsed() << "Assigning hero " << hero.id << " to attack threat at " << p << endl;
                    actions[h] = assignMoveAction(hero, p);
                }
                if (allActionsAreSet(actions))
                    return actions;
            }
            else {
                cerr << "Determined threat cannot be eliminated " << threat << endl;
            }
        }


        for (int h = 0; h < heroes.size(); h++) {
            if (actions[h]) {
                cerr << elapsed() << "Hero " << h << " is already assigned an action (" << actions[h]->toString() << ")" << endl;
                continue;
            }
            Point placement;

            for (int r = 0; r < rings.size(); r++) {
                Ring<vector<Entity*>>& ring = rings[r];

                vector<Entity*> closest = findClosestMonsters(*heroes[h], ring.data, permitted_range[h]);
                if (!closest.empty()) {
                    bool canReachMultiple = false;
                    for (int m = min(4, (int)closest.size()); m > 0; m--) {
                        Point p = getMiddle(closest, m);
                        int reached = monstersReachedFrom(p);
                        if (reached >= m) {
                            placement = p;
                            canReachMultiple = true;
                            cerr << elapsed() << "Assigning hero " << h << " to location where it hits multiple monsters (" << reached << ") in ring " << r << " (" << placement << ")" << "\n";
                            break;
                        }
                    }
                    if (canReachMultiple)
                        break;
                    int min_distance;
                    const Entity* closest;
                    if (r == 0) {
                        // In ring 0 we don't care about segments.
                        if (!turnsToReachBase.empty() && turnsToReachBase[0].second < 6) {
                            closest = turnsToReachBase[0].first;
                        }
                        else {
                            closest = findClosestMonster(*heroes[h], ring.data, permitted_range[h], min_distance);
                        }
                    }
                    else {
                        closest = findClosestMonster(*heroes[h], ring.segmentList[h].data, permitted_range[h], min_distance);
                        if (closest == nullptr)
                            closest = findClosestMonster(*heroes[h], ring.data, permitted_range[h], min_distance, closest);
                    }
                    if (closest != nullptr) {
                        placement = closest->position;
                        cerr << elapsed() << "Assigning hero " << h << " to the closest monster " << closest->id << " in ring " << r << " (" << placement << ")" << "\n";
                        break;
                    }
                }
                else if (r == rings.size() - 1) {
                    placement = default_hero_location[h];
                    cerr << elapsed() << "Assigning hero " << h << " its default position (" << placement << ")" << "\n";
                }
                //}
            }
            // determine number of monsters attacked at current position and number attacked at new position.
            // if they are the same, consider casting a spell to redirect monsters towards my opponent.
            int currentReached = monstersReachedFrom(heroes[h]->position);
            int potentialReached = monstersReachedFrom(placement);

            if (potentialReached < currentReached) {
                cerr << elapsed() << "Aborting move -- reaches fewer monsters than current position." << endl;
                actions[h] = make_unique<WaitAction>();
            }
            else if (potentialReached == currentReached && my_stats.mana > 20 && inAttackMode) {
                // do control actions work inside my base?
                cerr << elapsed() << "Searching for monster to control" << endl;
                actions[h] = findMonsterToControl(*heroes[h], actions);
            }
            if (!actions[h]) {
                actions[h] = assignMoveAction(*heroes[h], placement);
            }
        }
        return actions;
    }

    Point getMiddle(vector<Entity*> monsters, int count) {
        Point p{ 0, 0 };
        count = min((int)monsters.size(), count);
        if (count == 0) {
            cerr << "Error: can't find middle of an empty list!" << endl;
            throw exception();
        }
        for (int i = 0; i < count; i++) {
            p.x += monsters[i]->position.x;
            p.y += monsters[i]->position.y;
        }
        p.x = p.x / count;
        p.x = p.y / count;
        return p;
    }

    int monstersReachedFrom(const Point& p) {
        int reached = 0;
        for (auto& entity : monsters) {
            if ((p - entity->position).distance() < ATTACK_RADIUS) {
                reached++;
            }
        }
        return reached;
    }

    int healthyMonstersReachedFrom(const Point& p) {
        int reached = 0;
        for (auto& entity : monsters) {
            if (entity->health > 8 && (p - entity->position).distance() < ATTACK_RADIUS) {
                reached++;
            }
        }
        return reached;
    }

    unique_ptr<ControlSpellAction> findMonsterToControl(const Entity& hero, const vector<unique_ptr<Action>>& actions) {
        vector<Entity*> monstersToControl;
        for (auto& entity : monsters) {
            if (entity->type == EntityType::Monster && entity->shield_life == 0 && entity->health > 10) {
                bool monsterIsControlled = entity->is_controlled;
                if (!monsterIsControlled) {
                    for (auto& a : actions) {
                        if (a && a->type == ActionType::CONTROL && static_cast<ControlSpellAction*>(a.get())->targetId == entity->id) {
                            monsterIsControlled = true;
                            break;
                        }
                    }
                }
                if (!monsterIsControlled) {
                    int monsterDist = (hero.position - entity->position).distance();
                    if (!(entity->willTargetOpponentBase() || entity->targettingOpponentBase()) && monsterDist > ATTACK_RADIUS&& monsterDist < CONTROL_SPELL_RANGE)
                        monstersToControl.push_back(entity);
                }
            }
        }
        if (monstersToControl.empty())
            return nullptr;

        sort(begin(monstersToControl), end(monstersToControl), [](const auto& a, const auto& b) -> bool { return a->health > b->health; });
        cerr << "Monsters to control: (" << monstersToControl.size() << ")" << endl;
        for (auto entity : monstersToControl) {
            cerr << *entity << endl;
        }
        return make_unique<ControlSpellAction>(*monstersToControl[0], alternatingPointAlongOpponentBase());
    }

    Point alternatingPointAlongOpponentBase() {
        static int controlCount = 0;
        double angle;
        if (controlCount % 2 == 0) {
            angle = opponent_stats.arc.start + (M_PI_2 / 5.0);
        }
        else {
            angle = opponent_stats.arc.end - (M_PI_2 / 5.0);
        }

        controlCount++;
        //cerr << elapsed() << "Sending spider to " << Polar(BASE_RADIUS, angle) << " (angle: " << angle << ") which is " << Polar(BASE_RADIUS, angle).toPoint(opponent_stats.base) << " (opponent base start/end angles are: " << opponent_stats.arc.start << " (" << radiansToDegrees(opponent_stats.arc.start) << "), " << opponent_stats.arc.end << " (" << radiansToDegrees(opponent_stats.arc.end) << "))" << endl;
        //cerr << elapsed() << "  sin(angle) * radius = " << sin(angle) * BASE_RADIUS << endl;
        return Polar(BASE_RADIUS, angle).toPoint(opponent_stats.base);
    }

    vector<Entity*> findClosestMonsters(const Entity& hero, const vector<Entity*>& monsters, const Arc& allowed_range)
    {
        vector<Entity*> closest;
        for (auto& monster : monsters) {
            // don't attack monsters that are going to attack my opponent
            if (monster->willTargetOpponentBase() || monster->targettingOpponentBase())
                continue;

            int distance = (hero.position - monster->position).distance();
            int turns_to_reach = countTurnsToInterdict(*monster, hero);
            if (!monster->isAliveInNTurns(turns_to_reach))
                continue;

            int base_distance = (my_stats.base - monster->position).distance();
            if (base_distance > BASE_RADIUS && !allowed_range.includes(monster->polar))
                continue;

            closest.push_back(monster);
        }

        sort(begin(closest), end(closest), [&hero](const auto& a, const auto& b) -> bool {
            int aDistance = (hero.position - a->position).distance();
            int bDistance = (hero.position - b->position).distance();
            return aDistance < bDistance;
            });
        //cerr << elapsed() << "Closest monsters" << endl;
        //for (auto e : closest) {
        //    cerr << elapsed() << "  " << e->id << ", dist: " << (hero.position - e->position).distance() << endl;
        //}
        return closest;
    }

    const Entity* findClosestMonster(const Entity& hero, const vector<Entity*>& monsters, const Arc& allowed_range, int& min_distance, const Entity* closest = nullptr)
    {
        for (auto& monster : monsters) {
            // don't attack monsters that are going to attack my opponent
            if (monster->willTargetOpponentBase() || monster->targettingOpponentBase())
                continue;

            int distance = (hero.position - monster->position).distance();
            int turns_to_reach = countTurnsToInterdict(*monster, hero);
            if (!monster->isAliveInNTurns(turns_to_reach))
                continue;

            int base_distance = (my_stats.base - monster->position).distance();
            if (base_distance > BASE_RADIUS && !allowed_range.includes(monster->polar))
                continue;

            cerr << elapsed() << "Determined it will take hero " << hero.id << " " << turns_to_reach << " turns to reach monster " << monster->id << endl;
            cerr << elapsed() << "  And this monster will have " << monster->healthInNTurns(turns_to_reach) << " health when the hero reaches it." << endl;

            if (closest == nullptr || distance < min_distance) {
                closest = monster;
                min_distance = distance;
            }
        }
        return closest;
    }
};

void init_top_left(PlayerStats& stats) {
    stats.arc.start = 0;
    stats.arc.end = M_PI_2;
    stats.base = { 0, 0 };
}

void init_top_right(PlayerStats& stats) {
    stats.arc.start = M_PI_2;
    stats.arc.end = M_PI;
    stats.base = { RIGHT_EDGE, 0 };
}

void init_bottom_right(PlayerStats& stats) {
    stats.arc.start = M_PI;
    stats.arc.end = M_PI * 3.0 / 2.0;
    stats.base = { RIGHT_EDGE, BOTTOM_EDGE };
}

void init_bottom_left(PlayerStats& stats) {
    stats.arc.start = M_PI * 3.0 / 2.0;
    stats.arc.end = M_PI * 2.0;
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

    cerr << "Base defense start angle: " << radiansToDegrees(my_stats.arc.start) << ", end angle: " << radiansToDegrees(my_stats.arc.end) << endl;

    double defender_arc = (my_stats.arc.end - my_stats.arc.start) / heroes_per_player;
    double current = my_stats.arc.start + defender_arc / 2.0;
    for (int i = 0; i < heroes_per_player; i++) {
        Point p = Polar(BASE_RADIUS + HERO_TRAVEL * (i == 1 ? 3 : 2), current).toPoint(my_stats.base);
        cerr << "Default hero position[" << i << "]: " << p << endl;
        default_hero_location.push_back(p);
        current += defender_arc;
    }
    double arc_for_two = (my_stats.arc.end - my_stats.arc.start) / 2;

    permitted_range.push_back({ my_stats.arc.start, my_stats.arc.start + arc_for_two });
    permitted_range.push_back({ my_stats.arc.start, my_stats.arc.end });
    permitted_range.push_back({ my_stats.arc.start + arc_for_two, my_stats.arc.end });

    current = opponent_stats.arc.start + defender_arc;
    for (int i = 0; i < heroes_per_player; i++) {
        Point p = Polar(BASE_RADIUS + HERO_TRAVEL * 3, current).toPoint(opponent_stats.base);
        default_attack_position.push_back(p);
        current += defender_arc / 2;
    }

    default_hero_location_in_attack_mode.push_back(Polar(BASE_RADIUS + HERO_TRAVEL * 2, my_stats.arc.start + arc_for_two / 2.0).toPoint(my_stats.base));
    default_hero_location_in_attack_mode.push_back(Polar(BASE_RADIUS + HERO_TRAVEL * 2, (opponent_stats.arc.start + opponent_stats.arc.end) / 2.0).toPoint(opponent_stats.base));
    default_hero_location_in_attack_mode.push_back(Polar(BASE_RADIUS + HERO_TRAVEL * 2, my_stats.arc.start + arc_for_two * 3.0 / 4.0).toPoint(my_stats.base));
    default_hero_location_in_standard_mode = default_hero_location;
}

// in Gold League now
int main()
{
    start = Clock::now();
    cin >> my_stats.base; cin.ignore();
    cerr << elapsed() << "Base: " << my_stats.base << endl;

    cin >> heroes_per_player; cin.ignore();
    cerr << elapsed() << "Heroes: " << heroes_per_player << endl;

    find_default_hero_placements(my_stats, opponent_stats);

    // game loop
    while (1) {
        turn_count++;
        cin >> my_stats;
        start = Clock::now();
        cerr << elapsed() << "Turn: " << turn_count << ", inAttackMode: " << inAttackMode << endl;
        cin >> opponent_stats;
        cerr << elapsed() << "My Stats: " << my_stats << endl;
        cerr << elapsed() << "Opponent Stats: " << opponent_stats << endl;

        int entity_count; // Amount of heros and monsters you can see
        cin >> entity_count; cin.ignore();

        cerr << elapsed() << "Entity Count: " << entity_count << endl;
        vector<Entity> entities(entity_count);

        ActionCalculator calculator(my_stats, opponent_stats, BASE_RADIUS, RING_SIZE, RINGS_TO_TRACK, heroes_per_player, my_stats.arc.start, my_stats.arc.end);

        //cerr << elapsed() << "Reading entities" << endl;
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

        cerr << elapsed() << "Known threats: " << endl;
        for (auto entity : calculator.threats) {
            cerr << elapsed() << "  " << entity->id << ", " << entity->position << endl;
        }

        cerr << elapsed() << "Known monsters: " << endl;
        for (auto entity : calculator.monsters) {
            if (find(begin(calculator.threats), end(calculator.threats), entity) == end(calculator.threats))
                cerr << elapsed() << "  " << entity->id << ", " << entity->position << endl;
        }
        calculator.calculateTurnsUntilIMonstersAttack();

        vector<unique_ptr<Action>> actions = calculator.determineActions();

        //cerr << elapsed() << "Actions determined" << endl;

        for (auto& action : actions) {
            cout << action->toString() << endl;
        }
    }
}