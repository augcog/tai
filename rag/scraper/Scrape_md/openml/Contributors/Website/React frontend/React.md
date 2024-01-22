# React App

### App structure

The structure of the source code looks as follows

```
App.js
index.js
components
|-- Sidebar.js
|-- Header.js
|-- ...
layouts
|-- Clear.js
|-- Main.js
pages
|-- auth
|-- cover
|-- docs
|-- search
routes
|-- index.js
|-- Routes.js
themes
```

The website is designed as a single-page application.
The top level files bootstrap the app. `index.js` simply renders the top component, and
`App.js` adds the relevant subcomponents based on the current theme and state.

`Routes.js` links components to the possible routes (based on the URL). The list of
possible routes is defined in `routes/index.js`.

`pages` contain the various pages of the website. It has subdirectories for:

- `auth`: All pages that require authorization (login). These routes are protected.
- `cover`: The front page of the website
- `docs`: All normal information pages (e.g. 'About', 'API',...)
- `search`: All pages related to searching for datasets, tasks, flows, runs, etc.

`layout` contains the possible layouts, `Main` or `Clear` (see below). You define the layout of a page by
adding its route to either `mainRoutes` or `clearRoutes` in `routes/index.js`. The default is the `Main` layout.

`themes` contains the overall theme styling for the entire website. Currently, there is a dark and a light theme. They can be set using `setTheme`
in the MainContext, see `App.js`.

#### Component structure

<img src="https://github.com/openml/docs/raw/master/docs/img/react-components.png" alt="components" width="80%"/>

The component structure is shown above, for the `Main` layout. The `App` component also holds the state of the website using
React's native Context API (see below). Next to the header and sidebar, the main component of the website (in yellow) shows
the contents of the current `page`. In this image, this is the search page, which has several subcomponents as explained below.

#### Search page

The search page is structured as follows:

- `SearchPanel`: the main search panel. Also contains callbacks for sorting and filtering, and lists what can be filtered or sorted on.

  - `FilterBar`: The top bar with the search statistics and functionality to add filters and sort results
  - `SearchResultsPanel`: The list of search results on the left. It shows a list of `Card` elements which are uniformly styled but their contents may vary. Depending on the selected type of result (selected in the left navigation bar) it is instantiated with different properties. E.g. a `DataListPanel` is a simple wrapper around `SearchResultsPanel` which defines the dataset-specific statistics to be shown in the cards.

    - Search tabs: The tabs that allow you to choose between different aspects of the results (Statistics, Overview (Dash)) or the different views on the selected dataset, task, etc. (Details, Analysis (Dash),...)
    - `ItemDetail`: When a search result is selected, this will show the details of the selection, e.g. the dataset details. Depending on the passed `type` prop, it will render the `Dataset`, `Task`, ... component.

The `api.js` file contains the `search` function, which translates a search query, filters, and other constraints into an ElasticSearch query and returns the results.

### Style guide

To keep a consistent style and minimize dependencies and complexity, we build on [Material UI](https://material-ui.com/) components and [FontAwesome](https://fontawesome.com) icons. Theming is defined in `themes/index.js` and loaded in as a context (`ThemeContext`) in `App.js`. More specific styling
is always defined through styled components in the corresponding pages.

#### Layouts

There are two top level layouts: `Main` loads the main layout with a `Sidebar`, `Header`,
and a certain page with all the contents. The `Clear.js` layout has no headers or sidebars,
but has a colored gradient background. It is used mainly for user login and registration or other quick forms.

The layout of the page content should use the [Material UI grid layout](https://material-ui.com/components/grid/). This
makes sure it will adapt to different device screen sizes. Test using your browsers development tools whether the layout
adapts correctly to different screens, including recent smartphones.

#### Styled components

Any custom styling (beyond the Material UI default styling) is defined in styled components which are defined within the file for each page.
Keep this as minimal as possible. Check if you can import styled components already defined for other pages, avoid duplication.

Styled div's are defined as follows:

```javascript
const OpenMLTitle = styled.div`
  color: white;
  font-size: 3em;
`;
```

Material UI components can be styled the same way:

```javascript
const WhiteButton = styled(Button)`
  display: inline-block;
  color: #fff;
`;
```

#### Color palette

We follow the general [Material UI color palette](https://material-ui.com/customization/color/#color) with shade 400, except when that doesn't give sufficient contrast. The main colors used (e.g. for the icons in the sidebar are: 'green[400]', 'yellow[700]', 'blue[800]', 'red[400]', 'purple[400]', 'orange[400]', 'grey[400]'. Backgrounds are generally kept white (or dark grey for the dark theme). The global context (see below) has a `getColor` function to get the colors of the search types, e.g. `context.getColor("run")` returns `red[400]`.

### Handling state

There are different levels of state management:

- Global state is handled via React's native Context API (we don't use Redux). Contexts are defined in the component tree where needed (usually higher up) by a context provider component, and is accessed lower in the component tree by a context consumer. For instance, see the `ThemeContext.Provider` in `App.js` and the `ThemeContext.Consumer` in `Sidebar.js`. There is a `MainContext` which contains global state values such as the logged in user details, and the current state of the search.
- Lower level components can pass state to their child components via props.
- Local state changes should, when possible, be defined by React Hooks.

Note that changing the global state will re-render the entire website. Hence, do this only when necessary.

#### State and search

Most global state variables have to do with search. The search pages typically work by changing the `query` and `filters` variables (see `App.js`). There is a `setSearch` function in the main context that can be called to change the search parameters. It checks whether the query has changed and whether updating the global state and re-rendering the website is necessary.

#### Lifecycle Methods

These are the React lifecycle methods and how we use them. When a component mounts, methods 1,2,4,7 will be called. When it updates, methods 2-6 will be called.

1. constructor(): Set the initial state of the components
2. getDerivedStateFromProps(props, state): Static method, only for changing the local state based on props. It returns the new state.
3. shouldComponentUpdate(nextProps, nextState): Decides whether a state change requires a re-rendering or not. Used to optimize performance.
4. render(): Returns the JSX to be rendered. It should NOT change the state.
5. getSnapshotBeforeUpdate(prevProps,prevState): Used to save 'old' DOM information right before an update. Returns a 'snapshot'.
6. componentDidUpdate(prevProps,prevState,snapshot): For async requests or other operations right after component update.
7. componentDidMount(): For async requests (e.g. API calls) right after the component mounted.
8. componentWillUnMount(): Cleanup before the component is destroyed.
9. componentDidCatch(error,info): For updating the state after an error is thrown.

#### Forms and Events

React wraps native browser events into _synthetic events_ to handle interactions in a cross-browser compatible way. After being wrapped, they are sent to
all event handlers, usually defined as callbacks. Note: for performance reasons, synthetic events are pooled and reused, so their properties are nullified after being consumed. If you want to use them asynchronously, you need to call `event.persist()`.

HTML forms are different than other DOM elements because they keep their own state in plain HTML. To make sure that we can control the state
we need to set the input field's `value` to a component state value.

Here's an example of using an input field to change the title displayed in the component.

```javascript
const titles: {mainTitle: 'OpenML'};

class App extends Component {
  this.state = {titles};

  // Receive synthetic event
  onTitleChange = (event) => {
    this.setState({titles.mainTitle : event.target.value});
  }

  render(){
    return (
      <div classname="App">
        <h1>{this.state.titles.mainTitle}</h1>
        <form>
          <input type="text"
          value={this.state.titles.mainTitle} // control state
          onChange={this.onTitleChange} // event handler callback
          />
        </form>
      </div>
    );
  }
}
```