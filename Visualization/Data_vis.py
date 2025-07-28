import streamlit as st
import pandas as pd

# Set page configuration
st.set_page_config(
    page_title="Boxing Analytics",
    page_icon="🥊",
    layout="wide"
)

# Create tabs for different pages
tab1, tab2 = st.tabs(["Data Visualization", "Punch Analysis"])

# First tab - Original visualization
with tab1:
    st.title("Data Visualization App")
    st.write("This is a simple Streamlit app for data visualization.")

    # Upload CSV file
    uploaded_file = st.file_uploader("Upload a CSV file", type=["csv"])

    if uploaded_file is not None:
        import matplotlib.pyplot as plt

        # Load data
        df = pd.read_csv(uploaded_file)
        st.write("### Data Preview")
        st.write(df.head())

        # Select column for visualization
        column = st.selectbox("Select a column to visualize", df.columns)

        # Plot histogram
        fig, ax = plt.subplots()
        df[column].hist(ax=ax, bins=20)
        st.pyplot(fig)

# Second tab - Punch Analysis
with tab2:
    st.title("Punch Analysis")
    
    # Create sample data
    punch_data = {
        "Punch Type": ["Jab", "Cross", "Hook", "Uppercut", "Overhand"],
        "Speed": [8.5, 9.2, 8.8, 7.5, 9.0],
        "Force": [75, 95, 85, 80, 90]
    }
    
    # Create DataFrame
    df_punches = pd.DataFrame(punch_data)
    
    # Display the table with some styling
    st.dataframe(
        df_punches,
        use_container_width=True,
        hide_index=True,
        column_config={
            "Punch Type": st.column_config.TextColumn(
                "Punch Type",
                width="medium",
            ),
            "Speed": st.column_config.NumberColumn(
                "Speed (m/s)",
                format="%.1f",
                width="small",
            ),
            "Force": st.column_config.NumberColumn(
                "Force (N)",
                format="%.0f",
                width="small",
            ),
        }
    )
    
    # Add some statistics
    st.subheader("Punch Statistics")
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.metric("Average Speed", f"{df_punches['Speed'].mean():.1f} m/s")
    with col2:
        st.metric("Average Force", f"{df_punches['Force'].mean():.0f} N")
    with col3:
        st.metric("Strongest Punch", f"{df_punches.loc[df_punches['Force'].idxmax(), 'Punch Type']}")
