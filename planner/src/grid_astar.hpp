#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <chrono>
#include <queue>
#include <list>
#include <map>
#include <unordered_map>


//template <int dim = 3, int noncyclic = 2>
#define dim 3
#define noncyclic 2
class grid_astar
{
public:
	template <class T>
		class reservable_priority_queue: public std::priority_queue<T>
	{
	public:
		typedef typename std::priority_queue<T>::size_type size_type;
		reservable_priority_queue(size_type capacity = 0)
		{
			reserve(capacity);
		};
		void reserve(size_type capacity)
		{
			this->c.reserve(capacity);
		} 
		size_type capacity() const
		{
			return this->c.capacity();
		}
		void clear()
		{
			this->c.clear();
		}
		void pop_back()
		{
			this->c.pop_back();
		}
	};
	class vec
	{
	public:
		int e[dim];

		vec(const int *v)
		{
			for(int i = 0; i < dim; i ++) e[i] = v[i];
		}
		vec(const float *v)
		{
			for(int i = 0; i < dim; i ++) e[i] = lroundf(v[i]);
		}
		vec()
		{
		}
		void cycle(int &v, const int c)
		{
			v = v % c;
			if(v < 0) v += c;
			if(v > c / 2) v -= c;
		}
		void cycle_unsigned(int &v, const int c)
		{
			v = v % c;
			if(v < 0) v += c;
		}
		bool operator ==(const vec& v) const
		{
			for(int i = 0; i < dim; i ++) if(v.e[i] != this->e[i]) return false;
			return true;
		}
		bool operator !=(const vec& v) const
		{
			return !(*this == v);
		}
		vec operator +(const vec& v) const
		{
			vec out = *this;
			for(int i = 0; i < dim; i ++)
			{
				out[i] += v[i];
			}
			return out;
		}
		vec operator -(const vec& v) const
		{
			vec out = *this;
			for(int i = 0; i < dim; i ++)
			{
				out[i] -= v[i];
			}
			return out;
		}
		int &operator [](const int& x)
		{
			return e[x];
		}
		const int &operator [](const int& x) const
		{
			return e[x];
		}
		void set(const int *init)
		{
			for(int i = 0; i < dim; i ++)
			{
				e[i] = init[i];
			}
		}
		int sqlen()
		{
			int out = 0;
			for(int i = 0; i < noncyclic; i ++)
			{
				out += e[i] * e[i];
			}
			return out;
		}
		float len()
		{
			return sqrtf(sqlen());
		}
		float norm()
		{
			float out = 0;
			for(int i = 0; i < dim; i ++)
			{
				out += powf(e[i], 2.0);
			}
			return sqrtf(out);
		}
	};
	class vecf
	{
	public:
		float e[dim];
		vecf(const float *v)
		{
			for(int i = 0; i < dim; i ++) e[i] = v[i];
		}
		vecf()
		{
		}
		float &operator [](const int& x)
		{
			return e[x];
		}
		const float &operator [](const int& x) const
		{
			return e[x];
		}
		vecf operator *(const vec& v) const
		{
			vecf out;
			for(int i = 0; i < dim; i ++)
			{
				out[i] = e[i] * v[i];
			}
			return out;
		}
		float sqlen()
		{
			float out = 0;
			for(int i = 0; i < noncyclic; i ++)
			{
				out += e[i] * e[i];
			}
			return out;
		}
		float len()
		{
			return sqrtf(sqlen());
		}
		float norm()
		{
			float out = 0;
			for(int i = 0; i < dim; i ++)
			{
				out += powf(e[i], 2.0);
			}
			return sqrtf(out);
		}
	};
	struct vec_hash
	{
		typedef std::size_t result_type;
		std::size_t rotl(const std::size_t x, const std::size_t n) const
		{
			return (x << n) | (x >> ((8 * sizeof(std::size_t)) - n));
		}
		std::size_t operator()(const vec& key) const
		{
			std::size_t hash = key.e[0];
			for(int i = 1; i < dim; i ++) 
				hash ^= rotl(key.e[i], 8 * sizeof(std::size_t) * i / dim);
			return hash;
		}
	};
	class pq
	{
	public:
		float p;
		float p_raw;
		vec v;
		pq()
		{
			p = 0;
		}
		pq(float p, float p_raw, vec v)
		{
			this->p = p;
			this->p_raw = p_raw;
			this->v = v;
		}
		bool operator<(const pq& b) const
		{
			// smaller first
			return this->p > b.p;
		}
	};
	template <class T, int block_width = 0x20>
		class gridmap
	{
	public:
		std::unique_ptr<T[]> c;
		vec size;
		vec block_size;
		size_t ser_size;
		size_t block_ser_size;
		size_t block_num;
		void clear(const T zero)
		{
			for(size_t i = 0; i < ser_size; i ++)
			{
				c[i] = zero;
			}
		}
		void clear_positive(const T zero)
		{
			for(size_t i = 0; i < ser_size; i ++)
			{
				if(c[i] >= 0) c[i] = zero;
			}
		}
		void reset(const vec &size)
		{
			this->size = size;

			for(int i = 0; i < noncyclic; i ++)
			{
				if(this->size[i] < (int)block_width) this->size[i] = block_width;
			}

			block_ser_size = 1;
			block_num = 1;
			for(int i = 0; i < dim; i ++)
			{
				int width;
				if(i < noncyclic)
				{
				   	width = block_width;
					block_size[i] = (this->size[i] + width - 1) / width;
				}
				else
				{
				   	width = this->size[i];
					block_size[i] = 1;
				}

				block_ser_size *= width;
				block_num *= block_size[i];
			}
			ser_size = block_ser_size * block_num;
			
			c.reset(new T[ser_size]);
		}
		gridmap(const vec &size)
		{
			reset(size);
		}
		gridmap()
		{
		}
		void block_addr(const vec &pos, size_t &baddr, size_t &addr)
		{
			addr = 0;
			baddr = 0;
			for(int i = 0; i < noncyclic; i ++)
			{
				addr *= block_width;
				addr += pos[i] % block_width;
				baddr *= block_size[i];
				baddr += pos[i] / block_width;
			}
			for(int i = noncyclic; i < dim; i ++)
			{
				addr *= size[i];
				addr += pos[i];
			}
		}
		T &operator [](const vec &pos)
		{
			size_t baddr, addr;
			block_addr(pos, baddr, addr);
			return c[baddr * block_ser_size + addr];
		}
	};

	gridmap<float> g;
	std::unordered_map<vec, vec, vec_hash> parents;
	reservable_priority_queue<pq> open;
	size_t queue_size_limit;

	void reset(const vec size)
	{
		g.reset(size);
		g.clear(FLT_MAX);
		parents.reserve(g.ser_size / 16);
		open.reserve(g.ser_size / 16);
	}
	grid_astar()
	{
		queue_size_limit = 0;
	}
	grid_astar(const vec size)
	{
		reset(size);
		queue_size_limit = 0;
	}
	void set_queue_size_limit(const size_t size)
	{
		queue_size_limit = size;
	}

	bool search(const vec &s, const vec &e, 
			std::list<vec> &path,
			std::function<float(const vec&, vec&)> cb_cost,
			std::function<float(const vec&, const vec&)> cb_cost_estim,
			std::function<std::vector<vec>&(const vec&, const vec&, const vec&)> cb_search,
			std::function<bool(const std::list<vec>&)> cb_progress,
			float progress_interval)
	{
		return search_impl(g, s, e, path,
				cb_cost, cb_cost_estim, cb_search, cb_progress,
				progress_interval);
	}
	bool search_impl(gridmap<float> &g,
			const vec &st, const vec &en, 
			std::list<vec> &path,
			std::function<float(const vec&, vec&)> cb_cost,
			std::function<float(const vec&, const vec&)> cb_cost_estim,
			std::function<std::vector<vec>&(const vec&, const vec&, const vec&)> cb_search,
			std::function<bool(const std::list<vec>&)> cb_progress,
			float progress_interval)
	{
		if(st == en)
		{
			return false;
		}
		auto s = st;
		auto e = en;
		for(int i = noncyclic; i < dim; i ++)
		{
			s.cycle_unsigned(s[i], g.size[i]);
			e.cycle_unsigned(e[i], g.size[i]);
		}
		g.clear(FLT_MAX);
		open.clear();
		parents.clear();

		g[s] = 0;
		open.push(pq(cb_cost_estim(s, e), 0, s));

		auto ts = std::chrono::high_resolution_clock::now();

		while(true)
		{
			//printf("search queue %d\n", (int)open.size());
			if(open.size() < 1)
			{
				// No fesible path
				printf("No fesible path\n");
				return false;
			}
			pq center = open.top();
			auto p = center.v;
			auto c = center.p_raw;
			open.pop();
			if(c > g[p]) continue;

			auto tnow = std::chrono::high_resolution_clock::now();
			if(std::chrono::duration<float>(tnow - ts).count() >= progress_interval)
			{
				std::list<vec> path_tmp;
				ts = tnow;
				find_path(s, p, path_tmp);
				cb_progress(path_tmp);
			}

			auto search_list = cb_search(p, s, e);
			int updates = 0;
			for(auto &diff: search_list)
			{
				auto next = p + diff;
				for(int i = noncyclic; i < dim; i ++)
				{
					next.cycle_unsigned(next[i], g.size[i]);
				}
				if((unsigned int)next[0] >= (unsigned int)g.size[0] ||
						(unsigned int)next[1] >= (unsigned int)g.size[1])
					continue;
				if(g[next] < 0) continue;

				auto cost = cb_cost(p, next);

				if(cost < 0)
				{
					continue;
				}
				auto cost_estim = cb_cost_estim(next, e);
				//printf(" - %d, %d, %d  c: %0.2f\n", next[0], next[1], next[2], cost);
				//printf("  - cost %0.3f, euclid %0.3f\n", cost, cost_estim);
				//if(cost < 0) exit(1);
				//printf("  - ok\n");

				if(g[next] > c + cost)
				{
					g[next] = c + cost;
					parents[next] = p;
					open.push(pq(c + cost + cost_estim, c + cost, next));
					if(queue_size_limit > 0 &&
							open.size() > queue_size_limit) open.pop_back();
					updates ++;
				}
			}
			if(updates == 0)
			{
				g[p] = -1;
			}
			if(g[e] != FLT_MAX)
			{
				break;
			}
			//printf("(parents %d)\n", (int)parents.size());
		}
		printf("AStar search finished (parents %d)\n", (int)parents.size());

		return find_path(s, e, path);
	}
	bool find_path(const vec &s, const vec &e, std::list<vec> &path)
	{
		vec n = e;
		while(true)
		{
			path.push_front(n);
			//printf("p- %d %d %d   %0.4f\n", n[0], n[1], n[2], g[n]);
			if(n == s) break;
			if(parents.find(n) == parents.end())
			{
				n = parents[n];
				//printf("px %d %d %d\n", n[0], n[1], n[2]);
				return false;
			}
			n = parents[n];
		}
		return true;
	}
};

